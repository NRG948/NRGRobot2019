package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.HatchClaw;

/**
 * A Subsystem to control Hatch claw to secure the hatches.
 */
public class HatchClawSubsystem extends Subsystem {

  public enum State{
    OPEN, CLOSE;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HatchClaw(State.OPEN));
  }

  public void setClawOpen(){
    RobotMap.hatchClawSolenoid.set(Value.kForward);
  }

  public void setClawClose(){
    RobotMap.hatchClawSolenoid.set(Value.kReverse);
  }
}
