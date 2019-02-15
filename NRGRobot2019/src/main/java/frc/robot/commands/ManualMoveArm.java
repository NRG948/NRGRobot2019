package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Commands to move the arm under manual control.
 */
public class ManualMoveArm extends Command {
  public ManualMoveArm() {
    requires(Robot.arm);
  }

  @Override
  protected void initialize() {
    System.out.println("ManualMoveArm init");
  }

  @Override
  protected void execute() {
    double upSpeed = Robot.oi.getXboxRightTrigger();
    double downSpeed = Robot.oi.getXboxLeftTrigger();
    double speed = upSpeed - downSpeed;
    Robot.arm.moveArm(speed);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.arm.stop();
    System.out.println("ManualMoveArm end");

  }

  @Override
  protected void interrupted() {
    end();
  }
}
