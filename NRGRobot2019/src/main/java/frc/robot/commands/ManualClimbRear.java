package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.MathUtil;

public class ManualClimbRear extends Command {

  double power;

  public ManualClimbRear(double power) {
    requires(Robot.climberRear);
    this.power = power;
  }

  @Override
  protected void initialize() {
    System.out.println("ManualClimberRear init");
  }

  @Override
  protected void execute() {
    if (Robot.oi.climberRearButton.get()) {
      double speed = Robot.oi.getXboxLeftY() * 0.5;
      Robot.climberRear.rawClimb(MathUtil.deadband(speed, 0.1));
    } else {
      Robot.climberRear.stop();
    }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    System.out.println("ManualClimberRear End");
    Robot.climberRear.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
