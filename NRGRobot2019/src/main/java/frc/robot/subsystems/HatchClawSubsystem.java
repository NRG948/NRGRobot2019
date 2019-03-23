package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.HatchClaw;

/**
 * A Subsystem to control Hatch claw to secure the hatches.
 */
public class HatchClawSubsystem extends Subsystem {
  private State state;
  
  public enum State {
    OPEN, CLOSE;
  }

  @Override
  public void initDefaultCommand() {
    state = State.OPEN;
    // setDefaultCommand(new HatchClaw(State.OPEN));
  }

  public void setClawOpen() {
    RobotMap.hatchClawSolenoid.set(Value.kForward);
    state = State.OPEN;
  }

  public void setClawClose() {
    RobotMap.hatchClawSolenoid.set(Value.kReverse);
    state = State.CLOSE;
  }

  public boolean isOpen() {
    return state == State.OPEN;
  }
}
