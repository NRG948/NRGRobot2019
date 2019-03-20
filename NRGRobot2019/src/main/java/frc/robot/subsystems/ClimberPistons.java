package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 *Extends or retracts the 2 front pistons
 */
public class ClimberPistons extends Subsystem {
  public void activate(boolean extend) {
    Value direction = extend ? Value.kForward : Value.kReverse;
    
//   RobotMap.climberSolenoid.set(direction);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
