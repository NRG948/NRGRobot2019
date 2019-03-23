package frc.robot.utilities;

import java.util.ArrayList;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.utilities.NRGPreferences.NumberPrefs;

/**
 * Add your docs here.
 */
public class VisionTargets {
  /**
   *
   */

  private static final double DISTANCE_THRESHOLD = 100.0;
  private static final String[] NO_TARGETS = new String[0];
  private static final double HALF_IMAGE_FOV = (Math.atan(36.0 / 57.125));
  private static final double DEFAULT_HALF_IMAGE_WIDTH = 480 / 2;
  private static final double TARGET_WIDTH_INCHES = 8.0;

  private ArrayList<TargetPair> targetPairs = new ArrayList<TargetPair>();
  private double imageCenterX;
  private int genCount;

  public void update() {
    this.imageCenterX = SmartDashboard.getNumber("Vision/imageCenterX", DEFAULT_HALF_IMAGE_WIDTH);
    this.genCount = (int) SmartDashboard.getNumber("Vision/genCount", this.genCount);

    ArrayList<TargetPair> newTargetPairs = new ArrayList<TargetPair>();
    String[] targetsJson = SmartDashboard.getStringArray("Vision/targetPairs", NO_TARGETS);
    GsonBuilder builder = new GsonBuilder();
    Gson gson = builder.create();
    for (int i = 0; i < targetsJson.length; i++) {
      newTargetPairs.add(gson.fromJson(targetsJson[i], TargetPair.class));
    }
    this.targetPairs = newTargetPairs;
  }

  public int getGenCount() {
    return this.genCount;
  }

  public boolean hasTargets() {
    return !this.targetPairs.isEmpty() && getDistanceToTarget() < DISTANCE_THRESHOLD;
  }

  private TargetPair getDesiredTargets() {
    return this.targetPairs.get(0);
  }

  public Point getCenterOfTargets() {
    return getDesiredTargets().getCenterOfTargets();
  }

  public double getAngleToTarget() {
    double centerX = getCenterOfTargets().x;
    double deltaX = centerX - imageCenterX;
    return Math.toDegrees(Math.atan2(deltaX, imageCenterX / Math.tan(HALF_IMAGE_FOV)));
  }

  // returns the targets position in the feild of view, normalized to a range of 1
  // to -1.
  // this is the same as "zeta" in the 2017 robot.
  public double getNormalizedTargetPosition() {
    double centerX = getCenterOfTargets().x;
    double deltaX = centerX - imageCenterX;
    return deltaX / imageCenterX;
  }

  // public double getHeadingToTarget() {
  // return RobotMap.navx.getAngle() + getAngleToTarget() / 12;
  // }

  public double getDistanceToTarget() {
    TargetPair desiredTarget = getDesiredTargets();
    double targetWidth = (desiredTarget.right.getMinX().x - desiredTarget.left.getMaxX().x);
    return (TARGET_WIDTH_INCHES * imageCenterX / (targetWidth * Math.tan(HALF_IMAGE_FOV)))
        * NumberPrefs.CAMERA_DISTANCE_SCALE.getValue();
  }
}
