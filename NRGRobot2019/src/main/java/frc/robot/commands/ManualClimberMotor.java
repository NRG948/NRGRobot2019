package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.MathUtil;

public class ManualClimberMotor extends Command {

  double power;

  public ManualClimberMotor(double power) {
    requires(Robot.climberMotor);
    this.power = power;
  }

  @Override
  protected void initialize() {
    System.out.println("ManualClimberMotor init");
  }

  @Override
  protected void execute() {
    double speed = Robot.oi.getXboxLeftY()*0.4;
    Robot.climberMotor.rawClimb(MathUtil.deadband(speed, 0.1));
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.climberMotor.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
