package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Command that moves the arm with PID when manually controlled with xbox triggers.
 */

public class ManualMoveArmWithPID extends Command {
  /*calculation: 2600 ticks is full range of the arm
  * we want 1300 ticks per second, to move the whole arm in two seconds
  * that's 1300 ticks/second * 1 second/1000 milliseconds * 20 milliseconds = 26 ticks
  */
  private static final int MAX_TICKS_PER_CYCLE = 26;
  public ManualMoveArmWithPID() {
    requires(Robot.arm);
  }

  @Override
  protected void initialize() {
    System.out.println("ManualMoveArmWithPID init");
  }

  @Override
  protected void execute() {
    double upSpeed = Robot.oi.getXboxRightTrigger();
    double downSpeed = Robot.oi.getXboxLeftTrigger();
    double speed = upSpeed - downSpeed;
    int currentPosition = Robot.arm.getCurrentArmPosition();
    int newPosition = (int)(currentPosition + speed * MAX_TICKS_PER_CYCLE);
    Robot.arm.setSetpoint(newPosition);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    System.out.println("ManualMoveArmWithPID end");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
