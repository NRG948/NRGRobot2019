package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualMoveArm;
import frc.robot.commands.ManualMoveArmWithPID;
import frc.robot.commands.MoveArmTo;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.PreferenceKeys;
import frc.robot.utilities.SimplePIDController;

/**
 * Subsystem which controls the arm movement.
 */
public class Arm extends Subsystem {
	private static final int DEAD_BAND_RANGE = 100;
	public static final double DEFAULT_ARM_MAX_POWER = 0.5;
	public static final double DEFAULT_HOLD_ARM_LEVEL = 0.2;
	public static final double DEFAULT_ARM_P = 0.01;
	public static final double DEFAULT_ARM_I = 0.002;
	public static final double DEFAULT_ARM_D = 0.001;

	public static final int DEFAULT_ARM_STOWED_TICKS = 0;
	public static final int DEFAULT_ARM_ACQUIRE_CARGO_TICKS = 280;
	public static final int DEFAULT_ARM_CARGO_SHIP_TICKS = 918;
	public static final int DEFAULT_ARM_ROCKET_CARGO_LOW_TICKS = 680;
	public static final int DEFAULT_ARM_ROCKET_CARGO_MEDIUM_TICKS = 1228;
	public static final int DEFAULT_ARM_MAX_ANGLE_TICKS = 2600; // slightly smaller than actual range (max = 2670)
	public static final int DEFAULT_ARM_TICK_TOLORANCE = 10; // TODO : figure out a good value line 21-26
	public static final int DEFAULT_ARM_INVERSION_TICKS = 1680;
	public static final int DEFAULT_ARM_HATCH_MEDIUM_TICKS = 1800; //TBD
	public static final int DEFAULT_ARM_LEVEL_TICKS = 750;

	private SimpleWidget pidOutputWidget;
	private SimpleWidget pidErrorWidget;
	private SimpleWidget pidSetpointWidget;

	private SimplePIDController pidController;

	public enum Angle {

		ARM_STOWED_ANGLE(PreferenceKeys.ARM_STOWED_TICKS, DEFAULT_ARM_STOWED_TICKS),
		ARM_ACQUIRE_CARGO_ANGLE(PreferenceKeys.ARM_ACQUIRE_CARGO_TICKS, DEFAULT_ARM_ACQUIRE_CARGO_TICKS),
		ARM_CARGO_SHIP_ANGLE(PreferenceKeys.ARM_CARGO_SHIP_TICKS, DEFAULT_ARM_CARGO_SHIP_TICKS),
		ARM_ROCKET_CARGO_LOW_ANGLE(PreferenceKeys.ARM_ROCKET_CARGO_LOW_TICKS, DEFAULT_ARM_ROCKET_CARGO_LOW_TICKS),
		ARM_ROCKET_CARGO_MEDIUM_ANGLE(PreferenceKeys.ARM_ROCKET_CARGO_MEDIUM_TICKS, DEFAULT_ARM_ROCKET_CARGO_MEDIUM_TICKS),
		ARM_MAX_ANGLE(PreferenceKeys.ARM_MAX_ANGLE_TICKS, DEFAULT_ARM_MAX_ANGLE_TICKS),
		ARM_INVERSION_ANGLE(PreferenceKeys.ARM_INVERSION_TICKS, DEFAULT_ARM_INVERSION_TICKS),
		ARM_HATCH_MEDIUM_ANGLE(PreferenceKeys.ARM_HATCH_MEDIUM_TICKS, DEFAULT_ARM_HATCH_MEDIUM_TICKS),
		ARM_FORWARD_ANGLE(PreferenceKeys.ARM_LEVEL_TICKS, DEFAULT_ARM_LEVEL_TICKS);
		
		public final String preferenceKey;
		public final int defaultTicks;

		private Angle(String prefKey, int defaultTicks) {
			this.preferenceKey = prefKey;
			this.defaultTicks = defaultTicks;
		}

		public int getTicks() {
			return Robot.preferences.getInt(preferenceKey, defaultTicks);
		}
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ManualMoveArmWithPID());
	}

	// Positive power means up from the starting (stowed) position.
	public void moveArm(double power) {
		rawMoveArm(power);
	}

	private void rawMoveArm(double power) {
		if (power < 0) {
			if (atBackLimit()) {
				power = 0;
			}
		} else {
			if (atFrontLimit()) {
				power = 0;
			}
		}
		if (power != 0) {
			// power = adjustPowerForGravity(power);
			RobotMap.armMotor.set(power);
		} else {
			stop();
		}
	}

	// Limit max power when the arm is moving toward the floor.
	private double adjustPowerForGravity(double power) {
		double maxPower = Robot.preferences.getDouble(PreferenceKeys.ARM_MAX_POWER, DEFAULT_ARM_MAX_POWER);
		int position = getCurrentArmPosition();
		if (position <= Angle.ARM_INVERSION_ANGLE.getTicks()) {
			power = MathUtil.clamp(power, -0.5 * maxPower, maxPower);
		} else {
			power = MathUtil.clamp(power, -maxPower, 0.5 * maxPower);
		}
		return power;
	}

	public void stop() {
		RobotMap.armMotor.stopMotor();
	}

	public void armPIDControllerInit(double p, double i, double d, double setpoint, double tolerance) {
		double maxPower = Robot.preferences.getDouble(PreferenceKeys.ARM_MAX_POWER, DEFAULT_ARM_MAX_POWER);
		pidController = new SimplePIDController(p, i, d, true).setOutputRange(-maxPower, maxPower)
				.setAbsoluteTolerance(tolerance).setSetpoint(setpoint).start();
	}

	public void armAnglePIDInit(double setpoint, double tolerance) {
		double p = Robot.preferences.getDouble(PreferenceKeys.ARM_P_TERM, DEFAULT_ARM_P);
		double i = Robot.preferences.getDouble(PreferenceKeys.ARM_I_TERM, DEFAULT_ARM_I);
		double d = Robot.preferences.getDouble(PreferenceKeys.ARM_D_TERM, DEFAULT_ARM_D);
		armPIDControllerInit(p, i, d, setpoint, tolerance);
	}

	public void armAnglePIDInit() {
		Robot.arm.armAnglePIDInit(RobotMap.armEncoder.get(), Arm.DEFAULT_ARM_TICK_TOLORANCE);
	}

	public void armAnglePIDExecute() {
		int armTicks = RobotMap.armEncoder.get();
		double cosTheta = calculateCosineTheta(armTicks);
		pidController.setOutputRange(-0.25 + 0.4 * cosTheta, 0.25 + 0.4 * cosTheta);
		double feedForward = calculateFeedForward(cosTheta);
		double armPIDOutput = pidController.updateWithFeedForward(armTicks, feedForward);

		// if arm is stowed, don't run PID
		if (armTicks < DEAD_BAND_RANGE && armPIDControllerOnTarget()) {
			armPIDOutput = 0;
		}
		pidOutputWidget.getEntry().setDouble(armPIDOutput);
		pidErrorWidget.getEntry().setDouble(pidController.getError());
		rawMoveArm(armPIDOutput);
	}

	public void armAnglePIDEnd() {
		pidController = null;
		stop();
	}

	public boolean armPIDControllerOnTarget() {
		return pidController.onTarget();
	}

	public void setPIDOutputLimits(double maxSpeed) {
		double armMaxPower = Robot.preferences.getDouble(PreferenceKeys.ARM_MAX_POWER, DEFAULT_ARM_MAX_POWER);
		double power = Math.min(Math.abs(maxSpeed), armMaxPower);
		pidController.setOutputRange(-power, power);
	}

	public boolean atFrontLimit() {
		return !RobotMap.armFrontLimitSwitch.get();
	}

	public boolean atBackLimit() {
		return !RobotMap.armBackLimitSwitch.get();
	}

	public int getSetpoint() {
		return (int) pidController.getSetpoint();
	}

	public void setSetpoint(int setpointInTicks) {
		if (setpointInTicks > 0 && setpointInTicks < DEAD_BAND_RANGE
				&& setpointInTicks != Arm.Angle.ARM_STOWED_ANGLE.getTicks()) {
			setpointInTicks = DEAD_BAND_RANGE;
		}
		pidController.setSetpoint(setpointInTicks);
		pidSetpointWidget.getEntry().setDouble(setpointInTicks);
	}

	public int getCurrentArmPosition() {
		return RobotMap.armEncoder.get();
	}

	// https://www.chiefdelphi.com/t/smoothly-controlling-an-arm/343880
	// based on cheesy poofs comments about kf * cos(theta) on this post ^
	public double calculateFeedForward(double cosTheta) {
		return DEFAULT_HOLD_ARM_LEVEL * cosTheta;
	}

	// Assumes the forward horizontal arm position is 0 degrees, increasing CCW.
	public double calculateCosineTheta(int armPositionTicks) {
		double thetaInDegrees = 90.0 * (double) (armPositionTicks - DEFAULT_ARM_LEVEL_TICKS)
				/ (DEFAULT_ARM_INVERSION_TICKS - DEFAULT_ARM_LEVEL_TICKS);
		return Math.cos(Math.toRadians(thetaInDegrees));
	}

	public boolean isCameraInverted() {
		return RobotMap.armEncoder.getDistance() > Angle.ARM_INVERSION_ANGLE.getTicks();
	}

	public void initShuffleboard() {
		ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
		ShuffleboardLayout armLayout = armTab.getLayout("Arm", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
		armLayout.add("Encoder", RobotMap.armEncoder);
		armLayout.add("Back Limit", RobotMap.armBackLimitSwitch);
		pidSetpointWidget = armLayout.add("PID setpoint", 0.0);
		pidErrorWidget = armLayout.add("PID error", 0.0);
		pidOutputWidget = armLayout.add("PID output", 0.0);
		ShuffleboardLayout positionLayout = armTab.getLayout("Positions", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 3).withProperties(Map.of("Label position", "HIDDEN"));
		positionLayout.add("Stow", new MoveArmTo(Arm.Angle.ARM_STOWED_ANGLE));
		positionLayout.add("Acquire Cargo", new MoveArmTo(Arm.Angle.ARM_ACQUIRE_CARGO_ANGLE));
		positionLayout.add("Low Cargo" , new MoveArmTo(Arm.Angle.ARM_ROCKET_CARGO_LOW_ANGLE));
		positionLayout.add("Cargoship", new MoveArmTo(Arm.Angle.ARM_CARGO_SHIP_ANGLE));
		positionLayout.add("Medium Cargo", new MoveArmTo(Arm.Angle.ARM_ROCKET_CARGO_MEDIUM_ANGLE));
	}
}
