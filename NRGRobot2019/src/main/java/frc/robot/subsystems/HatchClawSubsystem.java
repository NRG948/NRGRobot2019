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

  public enum State {
    OPEN, CLOSE;
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new HatchClaw(State.OPEN));
  }

  public void setClawOpen() {
    RobotMap.hatchClawSolenoid.set(Robot.isPracticeBot() ? Value.kReverse : Value.kForward);
    SmartDashboard.putBoolean("Hatch Claw/State", true);
  }

  public void setClawClose() {
    RobotMap.hatchClawSolenoid.set(Robot.isPracticeBot() ? Value.kForward : Value.kReverse);
    SmartDashboard.putBoolean("Hatch Claw/State", false);
  }
}
