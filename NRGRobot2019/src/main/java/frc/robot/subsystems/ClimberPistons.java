package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Extends or retracts the 3 climber pistons.
 */
public class ClimberPistons extends Subsystem {

  public void activate(boolean extend) {
    Value direction = extend ? Value.kForward : Value.kReverse;

    RobotMap.climberSolenoid.set(direction);
  }

  @Override
  protected void initDefaultCommand() {
  }
}