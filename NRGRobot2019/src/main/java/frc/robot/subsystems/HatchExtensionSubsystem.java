package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.HatchClaw;
import frc.robot.commands.HatchExtension;

/**
 * A Subsystem to control the Extension of the hatch mechanism
 */
public class HatchExtensionSubsystem extends Subsystem {

  public enum State{
    EXTEND, RETRACT;
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new HatchExtension(State.RETRACT));
  }

  public void extend(){
    RobotMap.hatchExtensionSolenoid.set(Value.kForward);
  }
  
  public void retract(){
    RobotMap.hatchExtensionSolenoid.set(Value.kReverse);
  }
}
