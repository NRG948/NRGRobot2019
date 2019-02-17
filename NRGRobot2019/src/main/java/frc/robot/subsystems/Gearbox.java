package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.GearShift;

/**
 * A Subsystem to control gearbox shifting.
 */
public class Gearbox extends Subsystem {

  public enum Gear {
    HIGH, LOW;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new GearShift(Gear.LOW));
  }

  public void setHighGear(){
    // RobotMap.gearboxSolenoid.set(Value.kForward);
  }

  public void setLowGear(){
    // RobotMap.gearboxSolenoid.set(Value.kReverse);
  }
}
