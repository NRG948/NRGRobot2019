package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.PreferenceKeys;
import frc.robot.utilities.SimplePIDController;

public class DriveOnHeadingDistance extends Command {
  private double distanceToDrive;
  private double maxPower;
  private double heading;
  private double tolerance;
  private Point origin;
  private static final double GO_SLOW_INCHES = 12.0; 

  private SimplePIDController distancePID = new SimplePIDController(0, 0, 0);
  private int cyclesOnTarget;

  public DriveOnHeadingDistance(double heading, double distance, double maxPower) {
    this.requires(Robot.drive);
    this.heading = heading;
    this.distanceToDrive = distance;
    this.maxPower = maxPower;
  }

  @Override
  protected void initialize() {
    System.out.println("DriveOnHeadingDistance init heading: " + this.heading + " distance: " + this.distanceToDrive);
    this.tolerance = Robot.preferences.getDouble(PreferenceKeys.DISTANCE_TOLERANCE, Drive.DEFAULT_DISTANCE_TOLERANCE);
    Robot.drive.driveOnHeadingInit(this.heading);

    double p = Robot.preferences.getDouble(PreferenceKeys.DRIVE_P_TERM, Drive.DEFAULT_DISTANCE_DRIVE_P);
    double i = Robot.preferences.getDouble(PreferenceKeys.DRIVE_I_TERM, Drive.DEFAULT_DISTANCE_DRIVE_I);
    double d =Robot.preferences.getDouble(PreferenceKeys.DRIVE_D_TERM, Drive.DEFAULT_DISTANCE_DRIVE_D);
    this.distancePID.setPID(p, i, d)
      .setSetpoint(this.distanceToDrive)
      .setAbsoluteTolerance(this.tolerance)
      .start();
    this.origin = Robot.positionTracker.getPosition();
    this.cyclesOnTarget = 0;
  }

  @Override
  protected void execute() {
    double distanceTraveled = Robot.positionTracker.calculateDistance(this.origin);
    double factor = this.distancePID.update(distanceTraveled);
    double revisedPower = this.maxPower * factor;
    double error = distancePID.getError();
    SmartDashboard.putNumber("DistancePID/Error", error);
    if (Math.abs(error) < tolerance) {
      revisedPower = 0.0;
    } else if (Math.abs(error) < GO_SLOW_INCHES) {
      revisedPower = 0.3 * Math.signum(error);
    }
    SmartDashboard.putNumber("DistancePID/Revised Power", revisedPower);
    Robot.drive.driveOnHeadingExecute(revisedPower);
  }

  /** Finishes the command if the target distance has been reached */
  @Override
  protected boolean isFinished() {
    if (distancePID.onTarget()) {
      cyclesOnTarget++;
    } else {
      cyclesOnTarget = 0;
    }
    SmartDashboard.putNumber("DistancePID/Cycles On Target", cyclesOnTarget);
    return (cyclesOnTarget >= 6);
  }

  @Override
  protected void end() {
    Robot.drive.stopMotor();
    Robot.drive.driveOnHeadingEnd();
    System.out.println("DriveOnHeadingDistance end");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
