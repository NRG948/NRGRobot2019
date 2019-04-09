package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Command to turn to a certain heading.
 */
public class TurnToHeadingUsingOneSide extends Command {
  private final double DEFAULT_TURN_TOLERANCE = 5.0;

  private double desiredHeading; // the heading we want the robot to end at
  private double maxPower; // gives the maximum power the robot is gonna drive when the command is executed
  private boolean forward;//if true the robot was driving forward, else the robot was driving backwards

  public TurnToHeadingUsingOneSide(double desiredHeading, double maxPower, boolean forward) {
    this.desiredHeading = desiredHeading; // assigns the heading
    this.maxPower = Math.abs(maxPower); // assigns the power
    this.forward = forward;
    this.requires(Robot.drive); // requires the Drive subsystem
  }

  @Override
  protected void initialize() {
    RobotMap.compressor.stop();
    Robot.drive.turnToHeadingInit(this.desiredHeading, DEFAULT_TURN_TOLERANCE);
    // this gives in the angle into the command and intializes the command and gives
    // in the tolerance
    System.out.println("TurnToHeadingUsingOneSide Init Desired: " + this.desiredHeading + "Current: " + RobotMap.navx.getAngle());
  }

  @Override
  protected void execute() {
    // executes the turn at the maximum speed
    Robot.drive.turnToHeadingExecute(this.maxPower, false, this.forward);
  }

  @Override
  protected boolean isFinished() {
    return Robot.drive.turnToHeadingOnTarget(); // this command checks whether the robot is on target if not corrects
                                                // it.
  }

  @Override
  protected void end() {
    Robot.drive.turnToHeadingEnd(); // terminates the turn
    RobotMap.compressor.start();
    System.out.println("TurnToHeadingUsingOneSide End Heading: " + RobotMap.navx.getAngle());
  }

  @Override
  protected void interrupted() {
    end();
  }
}
