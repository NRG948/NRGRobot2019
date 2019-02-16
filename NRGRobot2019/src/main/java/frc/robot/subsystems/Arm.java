package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualMoveArm;
import frc.robot.utilities.PreferenceKeys;
import frc.robot.utilities.SimplePIDController;

/**
 * Subsystem, moves the arm.
 */
public class Arm extends Subsystem {
	public static final double ARM_UP_MAX_POWER = 0.35;
	public static final double ARM_DOWN_MAX_POWER = 0.35;
	public static final double DEFAULT_ARM_P = 0.005;
	public static final double DEFAULT_ARM_I = DEFAULT_ARM_P / 10;
	public static final double DEFAULT_ARM_D = 0;

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
    double maxPowerUp = Robot.preferences.getDouble(PreferenceKeys.ARM_UP_MAX_POWER, ARM_UP_MAX_POWER);
		double maxPowerDown = Robot.preferences.getDouble(PreferenceKeys.ARM_DOWN_MAX_POWER, ARM_DOWN_MAX_POWER);
		
		pidController = new SimplePIDController(p, i, d, true)
								.setOutputRange(-maxPowerDown, maxPowerUp)
								.setAbsoluteTolerance(tolerance)
								.setSetpoint(setpoint)
								.start();
  } 

  public void ArmAnglePIDInit(double setpoint, double tolerance) {
		double p = Robot.preferences.getDouble(PreferenceKeys.ARM_P_TERM, DEFAULT_ARM_P);
		double i = Robot.preferences.getDouble(PreferenceKeys.ARM_I_TERM, DEFAULT_ARM_I);
		double d = Robot.preferences.getDouble(PreferenceKeys.ARM_D_TERM, DEFAULT_ARM_D);
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

