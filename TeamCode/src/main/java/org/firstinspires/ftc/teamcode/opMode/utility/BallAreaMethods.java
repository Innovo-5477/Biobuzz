package org.firstinspires.ftc.teamcode.opMode.utility;

import static java.lang.Math.asin;
import static java.lang.Math.cos;
import static java.lang.Math.sin;


import org.firstinspires.ftc.teamcode.opMode.constants.CameraConstants;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.RotatedRect;

import java.util.List;


public class BallAreaMethods {
    static double fx = CameraConstants.fx;
    static double fy = CameraConstants.fy;
    static double cx = CameraConstants.cx;
    static double cy = CameraConstants.cy;

    public static double calculateExpectedArea(double bx, double by) {

        double [] txty = RayCastingMethods.getTxTy(bx, by);
        double tx = txty[0];
        double ty = txty[1];
        double distance = RayCastingMethods.getBallDistance(bx, by);

        double deltaT = Math.asin(1.45/distance);
        double edgeTxPositive = tx + deltaT;
        double edgeTxNegative = tx - deltaT;
        double edgeXPositive = Math.tan(edgeTxPositive) * fx + cx;
        double edgeXNegative = Math.tan(edgeTxNegative) * fx + cx;
        double pixelRadiusX = (edgeXPositive - edgeXNegative) / 2;

        double edgeTyPositive = ty + deltaT;
        double edgeTyNegative = ty - deltaT;
        double edgeYPositive = Math.tan(edgeTyPositive) * fy + cy;
        double edgeYNegative = Math.tan(edgeTyNegative) * fy + cy;
        double pixelRadiusY = (edgeYPositive - edgeYNegative) / 2;

        double ballArea = Math.PI*pixelRadiusX*pixelRadiusY;
        return ballArea;
    }
    public static double percentError(double contourArea, double expectedArea) {
        return Math.abs(contourArea - expectedArea) / expectedArea * 100;
    }

    public static double getLoss(List<ColorBlobLocatorProcessor.Blob> blobs) {
        int i = 0;
        double totalError = 0;
        for (ColorBlobLocatorProcessor.Blob b: blobs) {
            RotatedRect boxFit = b.getBoxFit();
//          double x = cx - (boxFit.center.x - cx); //THIS IS FLIPPED
//          double y = cy - (boxFit.center.y - cy);
            double x = boxFit.center.x;
            double y = boxFit.center.y;
            double contourArea = b.getContourArea();
            double expectedArea = calculateExpectedArea(x, y);
            double error = percentError(contourArea, expectedArea);
            totalError += error;
            i++;
        }
        double averageError = totalError/i;
        return averageError;
    }
}
