package frc.robot.utilities;

import java.util.ArrayList;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class VisionTargets {
    private static final String[] NO_TARGETS = new String[0];
    private static final double HALF_IMAGE_FOV = Math.toRadians(31.1);
    private static final double HALF_IMAGE_WIDTH = 160;
    private static final double TARGET_WIDTH = 8.0;
    public Target left;
    public Target right;

    public void update() {
        this.left = null;
        this.right = null;
        String[] targetsJson = SmartDashboard.getStringArray("Vision/targets", NO_TARGETS);
        ArrayList<Target> newTargets = new ArrayList<Target>();
        GsonBuilder builder = new GsonBuilder();
        Gson gson = builder.create();
        for (int i = 0; i < targetsJson.length; i++) {
            newTargets.add(gson.fromJson(targetsJson[i], Target.class));
        }
        if (newTargets.size() >= 2) {
            for (int i = 0; i < newTargets.size() - 1; ++i) {
                Target current = newTargets.get(i);
                if (current.getSide() == Target.Side.LEFT) {
                    Target nextTarget = newTargets.get(i + 1);
                    if (nextTarget.getSide() == Target.Side.RIGHT) {
                        this.left = current;
                        this.right = nextTarget;
                        break;
                    }
                }
            }
        }
    }

    public boolean hasTargets() {
        return this.left != null && this.right != null;
    }

    public Point getCenterOfTargets() {
        Point centerPoint = new Point();
        Point leftCenter = this.left.getCenter();
        Point rightCenter = this.right.getCenter();
        centerPoint.x = ((leftCenter.x + rightCenter.x)/2);
        centerPoint.y = ((leftCenter.y + rightCenter.y)/2);
        return centerPoint;
    }

    public double getAngleToTarget() {
        double centerX = getCenterOfTargets().x;
        double deltaX = centerX - HALF_IMAGE_WIDTH;
        return Math.toDegrees(Math.atan2(deltaX, HALF_IMAGE_WIDTH*Math.atan(HALF_IMAGE_FOV)));
    }

    public double getDistanceToTarget() {
        double targetWidth = (this.right.getMinX().x - this.left.getMaxX().x);
        return TARGET_WIDTH * 2 * HALF_IMAGE_WIDTH / (2 * targetWidth * Math.tan(HALF_IMAGE_FOV));
    }
}
