package org.firstinspires.ftc.teamcode.opMode.utility;


import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.firstinspires.ftc.teamcode.opMode.constants.CameraConstants;

public class RayCastingMethods {
    static double fx = CameraConstants.fx;
    static double fy = CameraConstants.fy;
    static double cx = CameraConstants.cx;
    static double cy = CameraConstants.cy;

    static double [] cameraPosition = CameraConstants.relativeCameraPose;
    static double pitch = CameraConstants.cameraPitch;
    static double ballPlaneHeight = CameraConstants.ballPlaneHeight;

    public static double [] getTxTy(double ballX, double ballY) {
        double Xn = (ballX- cx) / fx;
        double Yn = (ballY- cy) / fy;
        double tx = Math.atan(Xn);
        double ty = Math.atan(Yn);
        return new double[]{tx, ty};
    }

    public static double getBallDistance(double bx, double by) {
        double [] txty = getTxTy(bx, by);
        double tx = txty[0];
        double ty = txty[1];
        double Xn = Math.tan(tx);
        double Yn = Math.tan(ty);
        double [] directionVector = new double[]{Xn, Yn, 1};
        double [] normVector = getUnitVector(directionVector);
        double [] worldVector = pitchTransform(normVector, pitch);
        double height = cameraPosition[2];


        double lambda = (ballPlaneHeight-height) / worldVector[2];
        double xDistance = lambda*worldVector[0];
        double yDistance = lambda*worldVector[1];
        double [] displacementVector = new double[]{xDistance, yDistance, (height-ballPlaneHeight)};
        return getMagnitude(displacementVector);
    }

    public static double [] getBallPose(double bx, double by) {

        double [] txty = getTxTy(bx, by);
        double tx = txty[0];
        double ty = txty[1];
        double Xn = Math.tan(tx);
        double Yn = Math.tan(ty);
        double [] directionVector = new double[]{Xn, Yn, 1};
        double [] normVector = getUnitVector(directionVector);
        double [] worldVector = pitchTransform(normVector, pitch);
        double height = cameraPosition[2];


        double lambda = (ballPlaneHeight-height) / worldVector[2];
        double ballX = cameraPosition[0] + lambda*worldVector[0];
        double ballY = cameraPosition[1] + lambda*worldVector[1];
        return new double[]{ballX, ballY};

    }

    public static double getMagnitude(double [] vector) {
        double x = vector[0];
        double y = vector[1];
        double z = vector[2];


        return  Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    private static double [] getUnitVector (double [] vector) {
        double x = vector[0];
        double y = vector[1];
        double z = vector[2];

        double magnitude = getMagnitude(vector);
        if (magnitude != 0) {
            x /= magnitude;
            y /= magnitude;
            z /= magnitude;
            return new double[]{x, y, z};
        }
        else return vector;
    }

    private static double [] pitchTransform(double [] directionVector, double pitch) {
        double [][] rotationMatrix = getRotationMatrix(pitch);

        double [] finalArr = new double[3];

        for (int i = 0; i<3;i++) {
            for (int j = 0; j<3; j++) {
                finalArr[i] += directionVector[j] * rotationMatrix[i][j];
            }
        }
        return finalArr;
    }

    private static double [][] getRotationMatrix(double pitch) {
        return new double[][]{
                {1, 0, 0},
                {0, -sin(Math.toRadians(pitch)), cos(Math.toRadians(pitch))},
                {0, -cos(Math.toRadians(pitch)), -sin(Math.toRadians(pitch))}
        };
    }



}
