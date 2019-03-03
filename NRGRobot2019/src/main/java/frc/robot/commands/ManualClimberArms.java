package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberArms;
import frc.robot.utilities.MathUtil;

public class ManualClimberArms extends Command {
  public ManualClimberArms() {
    requires(Robot.climberArms);
  }

  @Override
  protected void initialize() {
    System.out.println("ManualClimbeArms Init");
  }

  @Override
  protected void execute() {
    if (Robot.oi.climberArmsButton.get()) {
      double speed = Robot.oi.getXboxLeftY() * 0.8;
      Robot.climberArms.rawClimb(MathUtil.deadband(speed, 0.1));
    } else {
      Robot.climberArms.stop();
    }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    System.out.println("ManualClimberArms End");
    Robot.climberArms.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
