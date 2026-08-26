package org.firstinspires.ftc.teamcode.opMode.utility;


import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class RayCastingMethods {

    public static double [] getBallPose(double Xn, double Yn, double[] cameraPosition, double pitch, double ballPlaneHeight) {
        double [] directionVector = new double[]{Xn, Yn, 1};
        double [] normVector = getUnitVector(directionVector);
        double [] worldVector = pitchTransform(normVector, pitch);
        double height = cameraPosition[2];


        double lambda = (ballPlaneHeight-height) / worldVector[2];
        double ballX = cameraPosition[0] + lambda*worldVector[0];
        double ballY = cameraPosition[1] + lambda*worldVector[1];
        return new double[]{ballX, ballY};

    }

    private static double [] getUnitVector (double [] matrix) {
        double x = matrix[0];
        double y = matrix[1];
        double z = matrix[2];

        double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
        x /= magnitude;
        y /= magnitude;
        z /= magnitude;
        return new double[]{x, y, z};
    }

    private static double [] pitchTransform(double [] directionVector, double pitch) {
        double [][] rotationMatrix = getRotationMatrix(pitch);

        for (int i = 0; i<3;i++) {
            for (int j = 0; j<3; j++) {
                if (rotationMatrix[i][j] != 0){
                    directionVector[i] *= rotationMatrix[i][j];
                }

            }
        }
        return directionVector;
    }

    private static double [][] getRotationMatrix(double pitch) {
        return new double[][]{
                {1, 0, 0},
                {0, sin(Math.toRadians(pitch)), cos(Math.toRadians(pitch))},
                {0, cos(Math.toRadians(pitch)), -sin(Math.toRadians(pitch))}
        };
    }



}
