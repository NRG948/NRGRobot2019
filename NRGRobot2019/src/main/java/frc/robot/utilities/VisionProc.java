package frc.robot.utilities;
import org.opencv.core.Point;

public class VisionProc {
    private Point leftCenter; 
    private Point rightCenter;
    private final double CENTER_PIXEL_LOCATION = 0; //need to figure out this value
    private final double KNOWN_FOCAL_LENGTH_PIXELS = 0; //need to figure out this value

    public VisionProc() {
        
    }

    public void setCenterPoints(Target leftTarget, Target rightTarget) {
        this.leftCenter = new Point((leftTarget.getMinX().x + leftTarget.getMaxX().x), (leftTarget.getMinY().y + leftTarget.getMaxY().y)/2);
        this.rightCenter = new Point((rightTarget.getMinX().x + rightTarget.getMaxX().x), (rightTarget.getMinY().y + rightTarget.getMaxY().y)/2);
    }

    public double getDistanceToCenter(Target leftTarget, Target rightTarget) {
        //assumes we are aligned straight towards the tapes, and that we can see both Targets
        this.setCenterPoints(leftTarget, rightTarget);
        double centerX = (leftCenter.x + rightCenter.x)/2;
        return (centerX - CENTER_PIXEL_LOCATION)/CENTER_PIXEL_LOCATION;
    }

    public double getAngleToTurn(Target leftTarget, Target rightTarget) {
        this.setCenterPoints(leftTarget, rightTarget); //figure out relative leftCenter/rightCenter, based on where we are
        if(leftTarget.getMaxY().y - leftTarget.getMinY().y > rightTarget.getMaxY().y - rightTarget.getMinY().y)
            return Math.toDegrees(Math.atan((leftCenter.x - CENTER_PIXEL_LOCATION) / KNOWN_FOCAL_LENGTH_PIXELS));
        return Math.toDegrees(Math.atan((rightCenter.x - CENTER_PIXEL_LOCATION) / KNOWN_FOCAL_LENGTH_PIXELS));
    }
}
