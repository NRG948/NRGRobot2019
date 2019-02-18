/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.MathUtil;

public class DriveToVisionTape extends Command {
  private static final double HATCH_STOP_DISTANCE = 16;
  private static final double CARGO_STOP_DISTANCE = 18 + HATCH_STOP_DISTANCE;
  private static final double SLOW_DOWN_DISTANCE = 15;
  private static final double MIN_DRIVE_POWER = 0.15;
  private static final double MAX_DRIVE_POWER = 0.5;

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
    Robot.drive.driveOnHeadingInit(Robot.visionTargets.getHeadingToTarget());
  }

  @Override
  protected void execute() {
    double targetHeading = Robot.visionTargets.getHeadingToTarget();
    this.targetDistance = Robot.visionTargets.getDistanceToTarget();
    double slowDownDistance = delivery.getStopDistance() + SLOW_DOWN_DISTANCE;
    double power = MathUtil.clamp(targetDistance/slowDownDistance, MIN_DRIVE_POWER, MAX_DRIVE_POWER);
    Robot.drive.driveOnHeadingExecute(power, targetHeading);
  }

  @Override
  protected boolean isFinished() {
    return targetDistance <= delivery.getStopDistance();
  }

  @Override
  protected void end() {
    Robot.drive.driveOnHeadingEnd();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
