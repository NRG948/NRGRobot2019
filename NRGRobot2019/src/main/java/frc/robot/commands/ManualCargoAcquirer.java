package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.MathUtil;

/**
 * Command for manual Cargo Acquirer.
 */
public class ManualCargoAcquirer extends Command {
  public ManualCargoAcquirer() {
    requires(Robot.cargoAcquirer);
  }

  @Override
  protected void initialize() {
    System.out.println("ManualCargoAcquirer init");
  }

  @Override
  protected void execute() {
    double speed = Robot.oi.getXboxRightY();
    Robot.cargoAcquirer.rawAcquire(MathUtil.deadband(speed, 0.1));
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.cargoAcquirer.stop();
    System.out.println("ManualCargoAcquirer End");

  }

  @Override
  protected void interrupted() {
    end();
  }
}
