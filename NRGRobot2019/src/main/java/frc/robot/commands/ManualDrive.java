package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.PreferenceKeys;

/**
 * Command to get the joysticks' states and send them to the tank drive.
 */
public class ManualDrive extends Command {
  private static final boolean DEFAULT_TELEOP_SQUARE_INPUTS = true;
  
  private boolean squareInputs;

  public ManualDrive() {
    requires(Robot.drive); // this uses drive subsystem
  }

  @Override
  protected void initialize() {
    System.out.println("Manual Drive Init");
    this.squareInputs = Robot.preferences.getBoolean(PreferenceKeys.TELEOP_SQUARE_INPUTS, DEFAULT_TELEOP_SQUARE_INPUTS);
  }

  @Override
  protected void execute() {
    // Get the joysticks' states and sending them to tank drive.
    double left = Robot.oi.getLeftJoystickY();
    double right = Robot.oi.getRightJoystickY();
    Robot.drive.tankDrive(left, right, this.squareInputs);
  }

  @Override
  protected boolean isFinished() {
    return false; // this command never terminates to keep driving the robot.
  }

  @Override
  protected void end() {
    Robot.drive.stopMotor();
    System.out.println("Manual Drive End");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
