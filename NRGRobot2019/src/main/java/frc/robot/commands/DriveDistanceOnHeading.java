package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.NRGPreferences;
import frc.robot.utilities.SimplePIDController;

/**
 *
 */
public class DriveDistanceOnHeading extends Command {

  private static final double MIN_DRIVE_POWER = 0.3;

  protected double heading;
  protected double distanceToDrive;
  private double maxPower;
  private double tolerance;
  private Point origin;
  private static final double GO_SLOW_INCHES = 12.0;

  private SimplePIDController distancePID = new SimplePIDController(0, 0, 0);
  private int cyclesOnTarget;

  public DriveDistanceOnHeading(double heading, double distance, double maxPower, double tolerance) {
    this.requires(Robot.drive);
    this.heading = heading;
    this.distanceToDrive = distance;
    this.maxPower = MathUtil.clamp(maxPower, -1.0, 1.0);
    this.tolerance = tolerance;
  }

  public DriveDistanceOnHeading(double heading, double distance, double maxPower) {
    this(heading, distance, maxPower, 0.0);

  }

  @Override
  protected void initialize() {
    System.out.println("DriveDistanceOnHeading init heading: " + this.heading + " distance: " + this.distanceToDrive);
    double tolerance = this.tolerance;
    if(tolerance == 0.0) {
      tolerance = NRGPreferences.NumberPrefs.DISTANCE_TOLERANCE.getValue();
    }

    Robot.drive.driveOnHeadingInit(this.heading);

    double p = NRGPreferences.NumberPrefs.DISTANCE_DRIVE_P_TERM.getValue();
    double i = NRGPreferences.NumberPrefs.DISTANCE_DRIVE_I_TERM.getValue();
    double d = NRGPreferences.NumberPrefs.DISTANCE_DRIVE_D_TERM.getValue();

    this.distancePID.setPID(p, i, d).setSetpoint(this.distanceToDrive).setAbsoluteTolerance(tolerance)
        .setOutputRange(-Math.abs(this.maxPower), Math.abs(this.maxPower)).start();
    this.origin = Robot.positionTracker.getPosition();
    this.cyclesOnTarget = 0;
  }

  @Override
  protected void execute() {
    double distanceTraveled = Robot.positionTracker.calculateDistance(this.origin);
    double power = this.distancePID.update(distanceTraveled) * Math.signum(this.maxPower);
    double error = distancePID.getError();

    if (error < 0.0) {
      power = -MIN_DRIVE_POWER * Math.signum(this.maxPower);
    }

    Robot.drive.driveOnHeadingExecute(power);

    SmartDashboard.putNumber("DistancePID/Error", error);
    SmartDashboard.putNumber("DistancePID/Revised Power", power);
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
    return (cyclesOnTarget >= 4);
  }

  @Override
  protected void end() {
    Robot.drive.stopMotor();
    Robot.drive.driveOnHeadingEnd();
    System.out.println(String.format("DriveDistanceOnHeading End x:%.1f y:%.1f", Robot.positionTracker.getX(),
        Robot.positionTracker.getY()));
  }

  @Override
  protected void interrupted() {
    end();
  }
}
