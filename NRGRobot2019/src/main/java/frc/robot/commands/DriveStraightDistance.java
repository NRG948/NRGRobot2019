package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Drives straight for a certain distance. Suppling a negative power would make
 * the robot go backwards.
 */
public class DriveStraightDistance extends Command {
  private double xOrigin;// a class variable
  private double yOrigin;
  private final double distance;// this is a constant
  private final double maxPower;
  private final boolean stopMotors;

  public DriveStraightDistance(double distance, double maxPower, boolean stopMotors) {
    this.requires(Robot.drive);// requires Robot.drive in order to Drive straight
    this.distance = distance;// sets the value of the distance that we want to travel
    this.maxPower = maxPower;// sets the maximum power
    this.stopMotors = stopMotors;
  }

  public DriveStraightDistance(double distance, double maxPower) {
    this(distance, maxPower, true);
  }

  @Override
  protected void initialize() {
    System.out.println("DriveStraightDistance Init");
    this.xOrigin = Robot.positionTracker.getX(); // gets our current X position from the poistion tracker command
    this.yOrigin = Robot.positionTracker.getY();// gets our current Y position from the poistion tracker command
    Robot.drive.driveOnHeadingInit(RobotMap.navx.getAngle()); // We are getting our current heading and putting it into
                                                              // driveOnHeadingInit to adjust our current heading
  }

  @Override
  protected void execute() {
    Robot.drive.driveOnHeadingExecute(this.maxPower);// excuting the command and puts in the maximum power that the
                                                     // robot is gonna run on
  }

  @Override
  protected boolean isFinished() {
    return Robot.positionTracker.calculateDistance(this.xOrigin, this.yOrigin) >= this.distance;
  }// this calculates the distance that we have traveled from origin in order to
   // figure out if the command
   // needs to be terminated or not and returns true or false true being it needs
   // to be terminated and false being it needs to be continued

  @Override
  protected void end() {
    if (stopMotors) {
      Robot.drive.driveOnHeadingEnd();
    }
    System.out.println("DriveStraightDistance End");
  } // terminated the command as the robot has reached the distance that needs to be
    // traveled or if it needs to be interrupted

  @Override
  protected void interrupted() {
    end();
  } // calls the end command
}
