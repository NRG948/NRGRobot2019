package frc.robot.utilities;
import org.opencv.core.Point;

import frc.robot.utilities.Target.Side;

public class VisionProc {
    private static Point leftCenter; 
    private static Point rightCenter;
    private static final double HALF_IMAGE_WIDTH = 160; //I think the image width for raspberry pi is 320px so 320px/2
    private static final double HALF_LATERAL_FIELD = 45; //need to figure out this value, by measurement or data sheet
    private static final Point CENTER_PIXEL_LOCATION = new Point(120, 160); //not exactly sure, but I think that camera is 240px by 320px

    public VisionProc() {
        
    }

    public static void setCenterPoints(Target leftTarget, Target rightTarget) {
        leftCenter = new Point(((leftTarget.getMinX().x + leftTarget.getMaxX().x)/2 + leftTarget.getMinX().x), (((leftTarget.getMinY().y + leftTarget.getMaxY().y)/2 + leftTarget.getMinY().y)));
        rightCenter = new Point(((rightTarget.getMinX().x + rightTarget.getMaxX().x)/2 + rightTarget.getMinX().x), (((rightTarget.getMinY().y + rightTarget.getMaxY().y)/2 + rightTarget.getMinY().y)));
    }

    //took this whole method out for now, need to redo it.
    /*public double getDistanceToCenter(Target leftTarget, Target rightTarget) {
        //assumes we are aligned straight towards the tapes, and that we can see both Targets
        this.setCenterPoints(leftTarget, rightTarget);
        double centerX = (leftCenter.x + rightCenter.x)/2;
        return (centerX - CENTER_PIXEL_LOCATION)/CENTER_PIXEL_LOCATION;
    }*/

    public static double getAngleToTurn(Target leftTarget, Target rightTarget) {
        setCenterPoints(leftTarget, rightTarget); //figure out relative leftCenter/rightCenter, based on where we are
        if(leftTarget.getSide() == Side.LEFT && rightTarget.getSide() == Side.RIGHT) {
            if(leftTarget.getMaxY().y - leftTarget.getMinY().y > rightTarget.getMaxY().y - rightTarget.getMinY().y)
                return Math.toDegrees(Math.atan2(Math.sqrt(Math.pow((CENTER_PIXEL_LOCATION.x - leftCenter.x),2) + Math.pow((CENTER_PIXEL_LOCATION.y - leftCenter.y),2)), HALF_IMAGE_WIDTH * Math.atan(HALF_LATERAL_FIELD)));
            return Math.toDegrees(Math.atan2(Math.sqrt(Math.pow((CENTER_PIXEL_LOCATION.x - rightCenter.x),2) + Math.pow((CENTER_PIXEL_LOCATION.y - rightCenter.y),2)), HALF_IMAGE_WIDTH * Math.atan(HALF_LATERAL_FIELD)));
        } else if(leftTarget.getSide() == Side.RIGHT && rightTarget.getSide() == Side.LEFT) {
            if(leftTarget.getMaxY().y - leftTarget.getMinY().y > rightTarget.getMaxY().y - rightTarget.getMinY().y)
                return Math.toDegrees(Math.atan2(Math.sqrt(Math.pow((CENTER_PIXEL_LOCATION.x - rightCenter.x),2) + Math.pow((CENTER_PIXEL_LOCATION.y - rightCenter.y),2)), HALF_IMAGE_WIDTH * Math.atan(HALF_LATERAL_FIELD)));
            return Math.toDegrees(Math.atan2(Math.sqrt(Math.pow((CENTER_PIXEL_LOCATION.x - leftCenter.x),2) + Math.pow((CENTER_PIXEL_LOCATION.y - leftCenter.y),2)), HALF_IMAGE_WIDTH * Math.atan(HALF_LATERAL_FIELD)));
        }
        return 0;
    }
}
