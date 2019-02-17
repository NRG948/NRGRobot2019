package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ManualDrive extends Command {
  public ManualDrive() {
    requires(Robot.drive); // this uses drive subsystem 
  }

  @Override
  protected void initialize() {
    System.out.println("Manual Drive Init");
  }

  @Override
  protected void execute() {
    // Get the joysticks' states and sending them to tank drive.
    double left = Robot.oi.getLeftJoystickY();
    double right = Robot.oi.getRightJoystickY();
    Robot.drive.tankDrive(left, right);
  }

  @Override
  protected boolean isFinished() {
    return false; // this command never terminates to keep driving the robot. 
  }

  @Override
  protected void end() {
    Robot.drive.stopMotor();
    System.out.println("Manual Drive End");
  }  

  @Override
  protected void interrupted() {
    end();
  }
}
