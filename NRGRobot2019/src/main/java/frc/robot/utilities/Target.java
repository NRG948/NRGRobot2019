package frc.robot.utilities;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

public class Target {
    public enum Side {
        LEFT, RIGHT, UNKOWN;
    }

    private Point minX;
    private Point maxX;
    private Point minY;
    private Point maxY;
    private Side side;
    private boolean isCameraInverted = false;

    public Target(MatOfPoint mat, boolean isCameraInverted) {
        this.isCameraInverted = isCameraInverted;
        Point[] points = mat.toArray();
        Point first = points[0];
        if (isCameraInverted) {
            first.x = -first.x;
        } else {
            first.y = -first.y;
        }
        minX = first;
        maxX = first;
        minY = first;
        maxY = first;

        for (int i = 1; i < points.length; ++i) {
            Point point = points[i];
            if (isCameraInverted) {
                point.x = -point.x;
            } else {
                point.y = -point.y;
            }
            if (point.x < minX.x) {
                minX = point;
            }
            if (point.y < minY.y) {
                minY = point;
            }
            if (point.x > maxX.x) {
                maxX = point;
            }
            if (point.y > maxY.y) {
                maxY = point;
            }
        }

        if (minX.y > maxX.y) {
            side = isCameraInverted ? Side.LEFT : Side.RIGHT;

        } else if (minX.y < maxX.y) {
            side = isCameraInverted ? Side.RIGHT : Side.LEFT;
        } else {
            side = Side.UNKOWN;
        }
    }

    public String toString() {
        return "Target: " + side.toString();
    }

    public Side getSide() {
        return this.side;
    }

    public Point getMinX() {
        return this.minX;
    }

    public Point getMinY() {
        return this.minY;
    }

    public Point getMaxX() {
        return this.maxX;
    }

    public Point getMaxY() {
        return this.maxY;
    }

    public Point getCenter() {
        return new Point((getMinX().x + getMaxX().x) / 2, (getMinY().y + getMaxY().y) / 2);
    }

    public MatOfPoint toMatOfPoint() {
        Point adjMinX = new Point();
        Point adjMinY = new Point();
        Point adjMaxX = new Point();
        Point adjMaxY = new Point();
        if (isCameraInverted) {
            adjMinX.x = -this.minX.x;
            adjMinX.y = this.minX.y;
            adjMaxX.x = -this.maxX.x;
            adjMaxX.y = this.maxX.y;
            adjMinY.x = -this.minY.x;
            adjMinY.y = this.minY.y;
            adjMaxY.x = -this.maxY.x;
            adjMaxY.y = this.maxY.y;
        } else {
            adjMinX.x = this.minX.x;
            adjMinX.y = -this.minX.y;
            adjMaxX.x = this.maxX.x;
            adjMaxX.y = -this.maxX.y;
            adjMinY.x = this.minY.x;
            adjMinY.y = -this.minY.y;
            adjMaxY.x = this.maxY.x;
            adjMaxY.y = -this.maxY.y;
        }
        return new MatOfPoint(adjMinX, adjMinY, adjMaxX, adjMaxY);
    }    
    
    public boolean getIsCameraInverted(){
        return this.isCameraInverted;
    }
}