package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Delay Command.
 */
public class DelaySeconds extends Command {
  Timer timer;
  double seconds;

  public DelaySeconds(double seconds) {
    this.seconds = seconds;
  }

  @Override
  protected void initialize() {
    System.out.println("DelaySeconds(" + seconds + ")");
    timer = new Timer();
    timer.start();
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return timer.hasPeriodPassed(seconds);
  }

  @Override
  protected void end() {
    System.out.println("DelaySeconds end");
    timer.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
