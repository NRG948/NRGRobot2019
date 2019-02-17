package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

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
    Robot.climberMotor.rawClimb(power);
  }

  @Override
  protected boolean isFinished() {
    return true;
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
