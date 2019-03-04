/*package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.PreferenceKeys;
import frc.robot.utilities.SimplePIDController;

public class DriveOnHeadingDistance extends Command {
  private double distance;
  private double maxPower;
  private double heading;
  private double tolerance;
  private Point origin;

  private SimplePIDController distancePID;
  private int cyclesOnTarget;

  public DriveOnHeadingDistance(double heading, double distance, double maxPower) {
    this.heading = heading;
    this.distance = distance;
    this.maxPower = maxPower;
  }

  @Override
  protected void initialize() {
    Robot.drive.driveOnHeadingInit(this.heading);

    double p = Robot.preferences.getDouble(PreferenceKeys.DRIVE_STRAIGHT_DISTANCE_P, 0.5);
    double i = Robot.preferences.getDouble(PreferenceKeys.DRIVE_STRAIGHT_DISTANCE_I, 0.01);
    double d = Robot.preferences.getDouble(PreferenceKeys.DRIVE_STRAIGHT_DISTANCE_D, 1.5);
    this.distancePID = new SimplePIDController(p, i, d).setSetpoint(setpoint).setAbsoluteTolerance(1.0);
    this.origin = Robot.positionTracker.getPosition();
  }

  @Override
  protected void execute() {
    double distanceTravelled = Robot.positionTracker.calculateDistance(this.origin);
    double factor = this.distancePID.update(this.distance - distanceTravelled);
    double revisedPower = this.maxPower * factor;
    double error = distancePID.getError();
    SmartDashboard.putNumber("Distance PID ERROR", error);
    if (Math.abs(error) < tolerance) {
      revisedPower = 0.0;
    } else if (Math.abs(error) < 1.0) {
      revisedPower = 0.3 * Math.signum(error);
    }
    SmartDashboard.putNumber("Revised Power DriveStraightDistance", revisedPower);
    drive.driveOnHeading(revisedPower, heading);
  }

  // Finishes the command if the target distance has been exceeded
  protected boolean isFinished() {
    if (Math.abs(distancePID.getError()) < tolerance) {
      cyclesOnTarget++;
    } else {
      cyclesOnTarget = 0;
    }
    SmartDashboard.putNumber("Cycles On Target", cyclesOnTarget);
    return (cyclesOnTarget >= 6);
    // return distancePID.getError() < tolerance;
  }

  protected void end() {
    distancePID.reset();
    distancePIDOutput = 0.0;
    drive.rawStop();
    drive.driveOnHeadingEnd();
  }

  protected void interrupted() {
    end();
  }

  public void pidWrite(double output) {
    distancePIDOutput = output;
  }

}
*/
