package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * A Subsystem to control gearbox shifting.
 */
public class Gearbox extends Subsystem {

  public enum Gear {
    HIGH, LOW;
  }

  public Gear state = Gear.HIGH;

  @Override
  public void initDefaultCommand() {
  }

  public void setHighGear() {
    RobotMap.gearboxSolenoid.set(Value.kForward);
    state = Gear.HIGH;
  }

  public void setLowGear() {
    RobotMap.gearboxSolenoid.set(Value.kReverse);
    state = Gear.LOW;
  }

  public void toggleGears(){
    if(state == Gear.HIGH){
      setLowGear();
    }else{
      setHighGear();
    }
  }
}
