package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualMoveArm;
import frc.robot.utilities.SimplePIDController;

/**
 * Subsystem, moves the arm.
 */
public class Arm extends Subsystem {
  private SimplePIDController pidController;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualMoveArm());
  }

  // Positive power means up from starting position
  public void moveArm(double power){
      rawMoveArm(power);
  }
  
  private void rawMoveArm(double power) {
    // RobotMap.armMotor.set(power);
  }

  public void stop() {
    // RobotMap.armMotor.stopMotor();
  }

  public void armPIDControllerInit(double p, double i, double d, double setpoint, double tolerance) {
    double maxPowerUp = 0.4; //Robot.preferences.getDouble(PreferenceKeys.LIFT_UP_MAX_POWER, LIFT_POWER_SCALE_UP);
		double maxPowerDown = 0.4; //Robot.preferences.getDouble(PreferenceKeys.LIFT_DOWN_MAX_POWER, LIFT_POWER_SCALE_DOWN);
		
		pidController = new SimplePIDController(p, i, d, true)
								.setOutputRange(-maxPowerDown, maxPowerUp)
								.setAbsoluteTolerance(tolerance)
								.setSetpoint(setpoint)
								.start();
  } 

  public void ArmAnglePIDInit(double setpoint, double tolerance) {
		double p = 0.005; //Robot.preferences.getDouble(PreferenceKeys.LIFT_P_TERM, DEFAULT_LIFT_P);
		double i = p/10; //Robot.preferences.getDouble(PreferenceKeys.LIFT_I_TERM, DEFAULT_LIFT_I);
		double d = 0; //Robot.preferences.getDouble(PreferenceKeys.LIFT_D_TERM, DEFAULT_LIFT_D);
		armPIDControllerInit(p, i, d, setpoint, tolerance);
  }
  
  public void ArmAnglePIDExecute() {
		double currentPIDOutput = pidController.update(RobotMap.armEncoder.getDistance());

		SmartDashboard.putNumber("Arm Angle PID/Error", pidController.getError());
		SmartDashboard.putNumber("Arm Angle PID/Output", currentPIDOutput);

		rawMoveArm(currentPIDOutput);
	}

	public void ArmAnglePIDEnd() {
		pidController = null;
		stop();
	}
	
	public boolean lifterPIDControllerOnTarget() {
		return pidController.onTarget();
	}
}

