package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimberRear;
import frc.robot.subsystems.ClimberPistons.State;

/**
 * Climber Pistons.
 */
public class ActivateClimberPistons extends Command {
  private State state;

  public ActivateClimberPistons(State state) {
    requires(Robot.climberPistons);
    this.state = state;
  }

  @Override
  protected void initialize() {
    System.out.println("Climber Pistons " + state);
  }

  @Override
  protected void execute() {
    Robot.climberPistons.setState(state);
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
    end();
  }
}
