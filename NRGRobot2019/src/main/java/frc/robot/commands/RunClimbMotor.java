package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RunClimbMotor extends Command {
  private double power;

  public RunClimbMotor(double power) {
    requires(Robot.climberMotor);
    this.power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("RunClimbMotor Init " + power);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.climberMotor.rawClimb(power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("RunClimbMotor end");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
