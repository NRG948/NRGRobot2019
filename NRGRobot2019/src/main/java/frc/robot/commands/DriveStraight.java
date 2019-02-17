package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Command to drive strait.
 */
public class DriveStraight extends Command {
  public DriveStraight() {
    this.requires(Robot.drive);
  }

  @Override
  protected void initialize() {
    System.out.println("Drive Strait Init");
    Robot.drive.driveOnHeadingInit(RobotMap.navx.getAngle ()); // getting the current driving angle from the gyro 
  }
  @Override
  protected void execute() {
    // we are executing driveOnHeadingExecute using the right y joystick to determine the speed.
    Robot.drive.driveOnHeadingExecute(Robot.oi.getRightJoystickY()); 
  }
    
  @Override
  protected boolean isFinished() {
    return false; // this command never finishes.

  }

  @Override
  protected void end() {
    Robot.drive.driveOnHeadingEnd(); // we terminate this command using driveOnHeadingEnd
    System.out.println("Drive Strait End");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
