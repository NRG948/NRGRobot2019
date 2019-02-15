package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ManualMoveArm;

/**
 * Subsystem, moves the arm.
 */
public class Arm extends Subsystem {

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualMoveArm());
  }

  // Positive power means up from starting position
  public void moveArm(double power){
      rawMoveArm(power);
  }
  
  private void rawMoveArm(double power) {
    // RobotMap.armMotor.set(power);
  }

  public void stop() {
    // RobotMap.armMotor.stopMotor();
  }
}

