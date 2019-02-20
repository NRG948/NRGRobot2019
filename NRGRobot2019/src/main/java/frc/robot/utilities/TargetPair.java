package frc.robot.utilities;

import org.opencv.core.Point;

public class TargetPair {
    public Target left;
    public Target right;
    
    public TargetPair(Target left, Target right){
        this.left = left;
        this.right = right;
    }
    public Point getCenterOfTargets() {
        Point centerPoint = new Point();
        Point leftCenter = this.left.getCenter();
        Point rightCenter = this.right.getCenter();
        centerPoint.x = ((leftCenter.x + rightCenter.x) / 2);
        centerPoint.y = ((leftCenter.y + rightCenter.y) / 2);
        if (this.left.getIsCameraInverted()) {
            centerPoint.x = -centerPoint.x;
        } else {
            centerPoint.y = -centerPoint.y;
        }
        return centerPoint;
    }
}
