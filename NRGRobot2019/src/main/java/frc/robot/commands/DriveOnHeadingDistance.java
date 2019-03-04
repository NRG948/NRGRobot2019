/*package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.PreferenceKeys;
import frc.robot.utilities.SimplePIDController;

public class DriveOnHeadingDistance extends Command {
  public double distance;
  public double power;
  private double heading;
  private double tolerance;
  
  private SimplePIDController distancePID;
  private int cyclesOnTarget;

  
  public DriveOnHeadingDistance(double heading, double distance,double power)
  {	
    this.heading = heading;
    this.distance = distance;
    this.power = power;
  }

  
  @Override
  protected void initialize() {
    Robot.drive.driveOnHeadingInit(power);
    
    double p = Robot.preferences.getDouble(PreferenceKeys.DRIVE_STRAIGHT_DISTANCE_P, 0.5);
    double i = Robot.preferences.getDouble(PreferenceKeys.DRIVE_STRAIGHT_DISTANCE_I, 0.01);
    double d = Robot.preferences.getDouble(PreferenceKeys.DRIVE_STRAIGHT_DISTANCE_D, 1.5);
    distancePID.reset();
    distancePID.setOutputRange(-1, 1);
    distancePID.setAbsoluteTolerance(tolerance);
    distancePID.setSetpoint(distance);
    distancePID.setToleranceBuffer(6);
    distancePID.setPID(p,i,d);
    distancePID.enable();
  }

  @Override
  protected void execute() {
    SmartDashboard.putNumber("Distance PID OUTPUT", distancePIDOutput);
    double factor = MathHelper.clamp(distancePIDOutput, -1, 1);
    double revisedPower = power * factor;
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
    return(cyclesOnTarget >= 6);
//		return distancePID.getError() < tolerance;
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