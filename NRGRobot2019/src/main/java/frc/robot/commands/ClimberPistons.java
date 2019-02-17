package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber;

public class ClimberPistons extends Command {
private boolean extend;

  public ClimberPistons(boolean extend) {
    requires(new Climber());
    this.extend = extend;
  }

  @Override
  protected void initialize() {
    System.out.println("Climber Pistons");
  }

  @Override
  protected void execute() {
    Value direction = extend ? Value.kForward : Value.kReverse;
  
    RobotMap.climberSolenoid1.set(direction);
    RobotMap.climberSolenoid2.set(direction);
    RobotMap.climberSolenoid3.set(direction);
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
