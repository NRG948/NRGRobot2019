package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualCargoAcquirer;

/**
 * Subsystem for the Cargo Acquirer.
 */
public class CargoAcquirer extends Subsystem {
  public enum Direction {
    ACQUIRE, EJECT;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualCargoAcquirer());
  }

  public void acquire(double power, Direction direction){
    power = Math.abs(power);

    if (direction == Direction.ACQUIRE) {
      rawAcquire(-power);
    } else {
      rawAcquire(power);
  }
}
  
  public void rawAcquire(double power) {
    RobotMap.cargoAcquirerMotor.set(power);
  }

  public void stop() {
    RobotMap.cargoAcquirerMotor.stopMotor();
  }
}
