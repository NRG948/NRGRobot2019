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
    double heading = RobotMap.navx.getAngle();
    System.out.println(String.format("Drive Straight on heading %.1f", heading));
    Robot.drive.driveOnHeadingInit(heading); // getting the current driving angle from the gyro
  }
  @Override
  protected void execute() {
    // we are executing driveOnHeadingExecute using the left y joystick to determine the speed.
    Robot.drive.driveOnHeadingExecute(Robot.oi.getLeftJoystickY()); 
  }
    
  @Override
  protected boolean isFinished() {
    return false; // this command never finishes unless it is interrupted.
  }

  @Override
  protected void end() {
    Robot.drive.driveOnHeadingEnd(); // we terminate this command using driveOnHeadingEnd
    System.out.println("Drive Straight End");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
