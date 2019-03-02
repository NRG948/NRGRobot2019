package frc.robot.utilities;

import java.util.ArrayList;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class VisionTargets {
    private static final String[] NO_TARGETS = new String[0];
    private static final double HALF_IMAGE_FOV = Math.toRadians(31.1);
    private static final double DEFAULT_HALF_IMAGE_WIDTH = 480 / 2;
    private static final double TARGET_WIDTH_INCHES = 8.0;

    private ArrayList<TargetPair> targetPairs = new ArrayList<TargetPair>();
    private double imageCenterX;

    public void update() {
        ArrayList<TargetPair> newTargetPairs = new ArrayList<TargetPair>();
        imageCenterX = SmartDashboard.getNumber("Vision/imageCenterX", DEFAULT_HALF_IMAGE_WIDTH);
        String[] targetsJson = SmartDashboard.getStringArray("Vision/targetPairs", NO_TARGETS);
        GsonBuilder builder = new GsonBuilder();
        Gson gson = builder.create();
        for (int i = 0; i < targetsJson.length; i++) {
            newTargetPairs.add(gson.fromJson(targetsJson[i], TargetPair.class));
        }
        this.targetPairs = newTargetPairs;
    }

    public boolean hasTargets() {
        return !this.targetPairs.isEmpty();
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

    //returns the targets position in the feild of view, normalized to a range of 1 to -1.
    //this is the same as "zeta" in the 2017 robot.
    public double getNormalizedTargetPosition() {
        double centerX = getCenterOfTargets().x;
        double deltaX = centerX - imageCenterX;
        return deltaX / imageCenterX;
    }

    public double getHeadingToTarget() {
        return RobotMap.navx.getAngle() + getAngleToTarget()/4;
    }

    public double getDistanceToTarget() {
        TargetPair desiredTarget = getDesiredTargets();
        double targetWidth = (desiredTarget.right.getMinX().x - desiredTarget.left.getMaxX().x);
        return TARGET_WIDTH_INCHES * imageCenterX / (targetWidth * Math.tan(HALF_IMAGE_FOV));
    }
}
