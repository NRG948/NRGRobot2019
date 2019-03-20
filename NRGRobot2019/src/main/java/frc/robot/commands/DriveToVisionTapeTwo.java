package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.utilities.Deliver;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.PreferenceKeys;

public class DriveToVisionTapeTwo extends Command {
  private static final double CAMERA_SKEW = -0.6;
  private static final double SLOW_DOWN_DISTANCE = 9;
  public static final double DEFAULT_MIN_DRIVE_POWER = 0.15;
  public static final double DEFAULT_MAX_DRIVE_POWER = 0.65;

  private Deliver delivery;
  private double targetDistance;
  private double minDrivePower;
  private double maxDrivePower;
  private double targetHeading;
  private double power;
  private Point lastPosition = new Point();
  private boolean needsSlowdown = false;

  public DriveToVisionTapeTwo(Deliver delivery) {
    requires(Robot.drive);
    this.delivery = delivery;
  }

  @Override
  protected void initialize() {
    System.out.println("DriveToVisionTape init");

    this.minDrivePower = Robot.preferences.getDouble(PreferenceKeys.DRIVE_TO_VISION_TAPE_MIN_POWER,
        DEFAULT_MIN_DRIVE_POWER);
    this.maxDrivePower = Robot.preferences.getDouble(PreferenceKeys.DRIVE_TO_VISION_TAPE_MAX_POWER,
        DEFAULT_MAX_DRIVE_POWER);

    if (Robot.visionTargets.hasTargets()) {
      double desiredHeading = RobotMap.navx.getAngle() + Robot.visionTargets.getAngleToTarget() + CAMERA_SKEW;
      Robot.drive.driveOnHeadingInit(desiredHeading);
      this.targetDistance = Robot.visionTargets.getDistanceToTarget();
      System.out.println("targetDistance is : " + targetDistance + " heading is : " + desiredHeading);
    } else {
      Robot.drive.driveOnHeadingInit(Robot.drive.getCurrentHeading());
      this.targetDistance = 0;
    }

    double distanceToDrive = this.targetDistance - this.delivery.getStopDistance();
    this.needsSlowdown = distanceToDrive > 20;

    this.power = 0;
    this.lastPosition.x = Robot.positionTracker.getX();
    this.lastPosition.y = Robot.positionTracker.getY();
    this.targetHeading = Robot.drive.getCurrentHeading();
  }

  @Override
  protected void execute() {
    double distanceMoved = Robot.positionTracker.calculateDistance(this.lastPosition.x, this.lastPosition.y);
    this.targetDistance -= distanceMoved;
    double distanceRemaining = targetDistance - delivery.getStopDistance();
    SmartDashboard.putNumber("Vision/distanceRemaining", distanceRemaining);
    if (this.needsSlowdown) {
      power = MathUtil.clamp(distanceRemaining / SLOW_DOWN_DISTANCE, minDrivePower, maxDrivePower);
    } else {
      power = maxDrivePower;
    }
    Robot.drive.driveOnHeadingExecute(power);

    this.lastPosition.x = Robot.positionTracker.getX();
    this.lastPosition.y = Robot.positionTracker.getY();
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
