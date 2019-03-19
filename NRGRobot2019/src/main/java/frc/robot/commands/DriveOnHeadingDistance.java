package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.PreferenceKeys;
import frc.robot.utilities.SimplePIDController;

public class DriveOnHeadingDistance extends Command {
  /**
   *
   */

  private static final double MIN_DRIVE_POWER = 0.3
  ;
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

    double p = Robot.preferences.getDouble(PreferenceKeys.DISTANCE_DRIVE_P_TERM, Drive.DEFAULT_DISTANCE_DRIVE_P);
    double i = Robot.preferences.getDouble(PreferenceKeys.DISTANCE_DRIVE_I_TERM, Drive.DEFAULT_DISTANCE_DRIVE_I);
    double d =Robot.preferences.getDouble(PreferenceKeys.DISTANCE_DRIVE_D_TERM, Drive.DEFAULT_DISTANCE_DRIVE_D);
    this.distancePID.setPID(p, i, d)
      .setSetpoint(this.distanceToDrive)
      .setAbsoluteTolerance(this.tolerance)
      .setOutputRange(-Math.abs(this.maxPower), Math.abs(this.maxPower))
      .start();
    this.origin = Robot.positionTracker.getPosition();
    this.cyclesOnTarget = 0;
  }

  @Override
  protected void execute() {
    double distanceTraveled = Robot.positionTracker.calculateDistance(this.origin);
    double power = this.distancePID.update(distanceTraveled);
    double error = distancePID.getError();
    SmartDashboard.putNumber("DistancePID/Error", error);
    // if (error > 0 && error < tolerance) {
    //   revisedPower = 0.0;
    // } else
     if (error < 0) {
      power = MIN_DRIVE_POWER * Math.signum(error);
    }
    SmartDashboard.putNumber("DistancePID/Revised Power", power);
    Robot.drive.driveOnHeadingExecute(power);
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
