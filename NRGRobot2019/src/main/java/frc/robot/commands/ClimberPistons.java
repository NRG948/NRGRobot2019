/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber;

public class ClimberPistons extends Command {
private boolean extend;

  public ClimberPistons(boolean extend) {
    requires(new Climber());
    this.extend = extend;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Climber Piston Init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(extend){
      RobotMap.climberSolenoid1.set(Value.kForward);
      RobotMap.climberSolenoid2.set(Value.kForward);
      RobotMap.climberSolenoid3.set(Value.kForward);
    }
    else{
      RobotMap.climberSolenoid1.set(Value.kReverse);
      RobotMap.climberSolenoid2.set(Value.kReverse);
      RobotMap.climberSolenoid3.set(Value.kReverse);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
