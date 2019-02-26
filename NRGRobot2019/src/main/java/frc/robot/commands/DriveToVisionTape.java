package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.PreferenceKeys;

public class DriveToVisionTape extends Command {
  private static final double HATCH_STOP_DISTANCE = 16;
  private static final double CARGO_STOP_DISTANCE = 18 + HATCH_STOP_DISTANCE;
  private static final double SLOW_DOWN_DISTANCE = 15;
  public static final double DEFAULT_MIN_DRIVE_POWER = 0.15;
  public static final double DEFAULT_MAX_DRIVE_POWER = 0.7;

  public enum Deliver {
    Hatch(HATCH_STOP_DISTANCE), Cargo(CARGO_STOP_DISTANCE);

    private final double stopDistance;

    private Deliver(double stopDistance) {
      this.stopDistance = stopDistance;
    }

    public double getStopDistance() {
      return this.stopDistance;
    }
  }

  private Deliver delivery;
  private double targetDistance;

  public DriveToVisionTape(Deliver delivery) {
    requires(Robot.drive);
    this.delivery = delivery;
  }

  @Override
  protected void initialize() {
    System.out.println("DriveToVisionTape init");
    RobotMap.cameraLights.set(true);
    if (Robot.visionTargets.hasTargets()) {
      Robot.drive.driveOnHeadingInit(Robot.visionTargets.getHeadingToTarget());
      this.targetDistance = Double.MAX_VALUE;
    } else {
      this.targetDistance = 0;
    }
  }

  @Override
  protected void execute() {
    RobotMap.cameraLights.set(true);
    if (Robot.visionTargets.hasTargets()) {
      double targetHeading = Robot.visionTargets.getHeadingToTarget();
      this.targetDistance = Robot.visionTargets.getDistanceToTarget();
      double slowDownDistance = delivery.getStopDistance() + SLOW_DOWN_DISTANCE;
      double minDrivePower = Robot.preferences.getDouble(PreferenceKeys.DRIVE_TO_VISION_TAPE_MIN_POWER,
          DEFAULT_MIN_DRIVE_POWER);
      double maxDrivePower = Robot.preferences.getDouble(PreferenceKeys.DRIVE_TO_VISION_TAPE_MAX_POWER,
          DEFAULT_MAX_DRIVE_POWER);
      double power = MathUtil.clamp(targetDistance / slowDownDistance, minDrivePower, maxDrivePower);
      Robot.drive.driveOnHeadingExecute(power, targetHeading);
    }
  }

  @Override
  protected boolean isFinished() {
    return targetDistance <= delivery.getStopDistance();
  }

  @Override
  protected void end() {
    Robot.drive.driveOnHeadingEnd();
    RobotMap.cameraLights.set(false);
    System.out.println("DriveToVisionTape end");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
