package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.utilities.VisionTargetsApproach;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.NRGPreferences;

public class DriveToVisionTape extends Command {

  private static final double SLOW_DOWN_DISTANCE = 15;
  public static final double DEFAULT_MIN_DRIVE_POWER = 0.15;
  public static final double DEFAULT_MAX_DRIVE_POWER = 0.7;

  private VisionTargetsApproach delivery;
  private double targetDistance;
  private double minDrivePower;
  private double maxDrivePower;
  private double targetHeading;
  private double power;
  private Point lastPosition = new Point();

  public DriveToVisionTape(VisionTargetsApproach delivery) {
    requires(Robot.drive);
    this.delivery = delivery;
  }

  @Override
  protected void initialize() {
    System.out.println("DriveToVisionTape init");

    this.minDrivePower = NRGPreferences.NumberPrefs.DRIVE_TO_VISION_TAPE_MIN_POWER.getValue();
    this.maxDrivePower = NRGPreferences.NumberPrefs.DRIVE_TO_VISION_TAPE_MAX_POWER.getValue();

    if (Robot.visionTargets.hasTargets()) {
      Robot.drive.driveOnHeadingInit(getHeadingToTarget());
      this.targetDistance = Double.MAX_VALUE;
    } else {
      Robot.drive.driveOnHeadingInit(Robot.drive.getCurrentHeading());
      this.targetDistance = 0;
    }

    this.power = 0;
    this.lastPosition.x = Robot.positionTracker.getX();
    this.lastPosition.y = Robot.positionTracker.getY();
    this.targetHeading = Robot.drive.getCurrentHeading();
  }

  @Override
  protected void execute() {
    if (Robot.visionTargets.hasTargets()) {
      targetHeading = getHeadingToTarget();
      // double normalizedTargetPos =
      // Robot.visionTargets.getNormalizedTargetPosition();
      // targetHeading = Robot.drive.getCurrentHeading() + normalizedTargetPos;
      // SmartDashboard.putNumber("Vision/normalizedVisionTarget",
      // normalizedTargetPos);
      double distance = Robot.visionTargets.getDistanceToTarget();
      this.targetDistance = MathUtil.clamp(distance, 5, 100);
    } else {
      double distanceMoved = Robot.positionTracker.calculateDistance(this.lastPosition.x, this.lastPosition.y);
      this.targetDistance -= distanceMoved;
    }
    double distanceRemaining = targetDistance - delivery.getStopDistance();
    power = MathUtil.clamp(distanceRemaining / SLOW_DOWN_DISTANCE, minDrivePower, maxDrivePower);
    Robot.drive.driveOnHeadingExecute(power, targetHeading);

    this.lastPosition.x = Robot.positionTracker.getX();
    this.lastPosition.y = Robot.positionTracker.getY();
  }

  private double getHeadingToTarget() {
    return RobotMap.navx.getAngle() + Robot.visionTargets.getAngleToTarget() / 12;
  }

  @Override
  protected boolean isFinished() {
    return targetDistance <= delivery.getStopDistance();
  }

  @Override
  protected void end() {
    Robot.drive.driveOnHeadingEnd();
    System.out.println("DriveToVisionTape end");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
