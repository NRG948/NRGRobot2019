package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Command to move the arm under manual control, without PID.
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
    double upSpeed = Robot.oi.getXboxRightTrigger() * 0.5;
    double downSpeed = Robot.oi.getXboxLeftTrigger() * 0.5;
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
    System.out.println("ManualMoveArm End");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
