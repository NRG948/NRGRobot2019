package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Extends or retracts the 2 front pistons
 */
public class ClimberPistons extends Subsystem {
  public enum State {
    EXTEND, RETRACT;
  }

  public void setState(State state) {
    Value direction = state == State.EXTEND ? Value.kReverse : Value.kForward;
    RobotMap.climberSolenoid.set(direction);
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new ActivateClimberPistons(false));
  }
}
