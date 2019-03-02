package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimberRear;

/**
 * Climber Pistons.
 */
public class ActivateClimberPistons extends Command {
private boolean extend;

  public ActivateClimberPistons(boolean extend) {
    requires(Robot.climberArms);
    this.extend = extend;
  }

  @Override
  protected void initialize() {
    System.out.println("Climber Pistons " + extend);
  }

  @Override
  protected void execute() {
    //Robot.climberArms.activate(extend);
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
