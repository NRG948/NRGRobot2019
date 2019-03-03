package frc.robot.utilities;

/**
 * Add your docs here.
 */

import frc.robot.RobotMap;

public class PositionTracker {

  private double x;
  private double y;
  private double previousLeftEncoder;
  private double previousRightEncoder;
  private long previousTime = 0;
  private double maxVelocity;
  private double currentVelocity;

  public double getX() {
    return this.x;
  }

  public double getY() {
    return this.y;
  }

  public void setPosition(double x, double y) {
    this.x = x;
    this.y = y;
    this.previousLeftEncoder = RobotMap.driveLeftEncoder.getDistance();
    this.previousRightEncoder = RobotMap.driveRightEncoder.getDistance();
  }

  public void reset() {
    RobotMap.driveLeftEncoder.reset();
    RobotMap.driveRightEncoder.reset();
    setPosition(0, 0);
  }

  public void updatePosition() {
    double currentLeftEncoder = RobotMap.driveLeftEncoder.getDistance();
    double currentRightEncoder = RobotMap.driveRightEncoder.getDistance();
    double leftDelta = currentLeftEncoder - this.previousLeftEncoder;
    double rightDelta = currentRightEncoder - this.previousRightEncoder;
    double distance = (leftDelta + rightDelta) / 2;
    double heading = Math.toRadians(RobotMap.navx.getAngle());
    double deltaX = distance * Math.sin(heading);
    double deltaY = distance * Math.cos(heading);

    this.x += deltaX;
    this.y += deltaY;

    this.previousLeftEncoder = currentLeftEncoder;
    this.previousRightEncoder = currentRightEncoder;

    long currentTime = System.nanoTime();
    currentVelocity = distance / (currentTime - previousTime) * 1e9;
    maxVelocity = Math.max(currentVelocity, maxVelocity);
    previousTime = currentTime;
  }

  public double calculateDistance(double xOrigin, double yOrigin) {
    double deltaX = this.x - xOrigin;
    double deltaY = this.y - yOrigin;
    return Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
  }

  public double getCurrentVelocity() {
    return currentVelocity;
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }
}
