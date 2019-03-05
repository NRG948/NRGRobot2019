package frc.robot.utilities;

import edu.wpi.first.wpilibj.PIDBase.Tolerance;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * Add your docs here.
 */
public class SimplePIDController {
  private double kP; // factor for "proportional" control
  private double kI; // factor for "integral" control
  private double kD; // factor for "derivative" control

  private double minimumOutput = -1.0; // minimum output
  private double maximumOutput = 1.0; // maximum output
  private double minimumInput = -Double.MAX_VALUE; // minimum input - limit setpoint to this
  private double maximumInput = Double.MAX_VALUE; // maximum input - limit setpoint to this

  private PIDSource pidSource;
  private PIDOutput pidOutput;
  private boolean wasPIDReset = false; // is the pid controller enabled

  // the sum of the errors for use in the integral calc
  private double integral = 0.0;
  // the tolerance object used to check if on target
  private Tolerance tolerance;
  private double setpoint = 0.0;

  private double output = 0.0;
  private double prevInput;

  private double prevError = 0.0; // the prior error (used to compute derivative of error)
  private double prevTime;
  private boolean isIntegralNeededToHoldPosition;

  public SimplePIDController(double p, double i, double d, boolean isIntegralNeededToHoldPosition, PIDSource source,
      PIDOutput output) {
    this(p, i, d, isIntegralNeededToHoldPosition);

    this.pidSource = source;
    this.pidOutput = output;
  }

  public SimplePIDController(double p, double i, double d, boolean isIntegralNeededToHoldPosition) {
    setPID(p, i, d);
    this.tolerance = new Tolerance() {

      @Override
      public boolean onTarget() {
        throw new RuntimeException("tolerance needs to be specified explicitly");
      }
    };
    this.isIntegralNeededToHoldPosition = isIntegralNeededToHoldPosition;
  }

  public SimplePIDController(double p, double i, double d) {
    this(p, i, d, false);
  }

  /** Starts or resets the PID Controller. */
  public SimplePIDController start() {
    prevTime = System.nanoTime() / 1.0e9;
    this.wasPIDReset = true;
    integral = 0.0;
    return this;
  }

  public double update(double input, double setpoint) {
    this.setpoint = setpoint;
    return update(input);
  }

  public double update(double input) {
    return updateWithFeedForward(input, 0);
  }

  public double updateWithFeedForward(double input, double feedForward) {
    double currTime = System.nanoTime() / 1.0e9;
    double deltaTime = currTime - prevTime;

    input = MathUtil.clamp(input, minimumInput, maximumInput);
    double error = setpoint - input;

    if (wasPIDReset) {
      prevError = error;
      wasPIDReset = false;
    }

    double derivative = (error - prevError) / deltaTime;
    double pdfTerms = kP * error + kD * derivative + feedForward;

    // only integrate when close to setpoint
    if (pdfTerms >= maximumOutput) {
      output = maximumOutput;
      integral = 0;
    } else if (pdfTerms <= minimumOutput) {
      output = minimumOutput;
      integral = 0;
    } else {
      // integral is reset if sensor value overshoots the setpoint
      if (!isIntegralNeededToHoldPosition && Math.signum(error) != Math.signum(prevError)) {
        integral = 0;
      }
      integral += kI * (error + prevError) * 0.5 * deltaTime;
      output = MathUtil.clamp(pdfTerms + integral, minimumOutput, maximumOutput);
    }

    prevTime = currTime;
    prevError = error;
    prevInput = input;

    return output;
  }

  public void update() {
    double input = pidSource.pidGet();
    double result = update(input);
    pidOutput.pidWrite(result);
  }

  public SimplePIDController setPID(double p, double i, double d) {
    kP = p;
    kD = d;
    kI = i;
    return this;
  }

  public SimplePIDController setSetpoint(double setpoint) {
    this.setpoint = setpoint;
    wasPIDReset = true;
    return this;
  }

  public SimplePIDController setInputRange(double minimumInput, double maximumInput) {
    this.minimumInput = minimumInput;
    this.maximumInput = maximumInput;
    return this;
  }

  public SimplePIDController setOutputRange(double minimumOutput, double maximumOutput) {
    this.minimumOutput = minimumOutput;
    this.maximumOutput = maximumOutput;
    return this;
  }

  public SimplePIDController setAbsoluteTolerance(final double absoluteTolerance) {
    tolerance = new Tolerance() {
      private double tolerance = Math.abs(absoluteTolerance);

      @Override
      public boolean onTarget() {
        return Math.abs(setpoint - prevInput) <= tolerance;
      }
    };
    return this;
  }

  public double getError() {
    return prevError;
  }

  public double getOutput() {
    return output;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public boolean onTarget() {
    return tolerance.onTarget();
  }
}
