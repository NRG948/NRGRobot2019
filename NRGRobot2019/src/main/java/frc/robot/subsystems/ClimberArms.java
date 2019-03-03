package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ManualClimberArms;

/**
 * Extends or retracts the front climber arms.
 */
public class ClimberArms extends Subsystem {

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new ManualClimberArms());
  }

  public void rawClimb(double power) {
    RobotMap.climberArmsMotor.set(power);
  }

  public void stop() {
    RobotMap.climberArmsMotor.stopMotor();
  }
}