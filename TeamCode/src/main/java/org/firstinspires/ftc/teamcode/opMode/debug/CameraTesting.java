package org.firstinspires.ftc.teamcode.opMode.debug;

import android.util.Size;

import org.firstinspires.ftc.teamcode.opMode.constants.CameraConstants;
import org.firstinspires.ftc.teamcode.opMode.utility.BallAreaMethods;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opMode.utility.RayCastingMethods;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;
import java.util.concurrent.TimeUnit;

import dev.nextftc.ftc.ActiveOpMode;

@TeleOp(name = "Gain & Exposure Testing")
@Configurable
public class CameraTesting extends LinearOpMode {

    VisionPortal visionPortal;

    AprilTagProcessor aprilTag;
    ColorBlobLocatorProcessor colorLocatorProcessor;


    public static int constantGainNumber = 255;
    public static int constantExposureTime = 15;

    private long minExp;
    private long maxExp;

    private int minGain;
    private int maxGain;

    public int id = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //Change
        int gainNumber = 85;
        int exposureTime = 3;
        double frameNum = 0;

        double circularityThreshold = 0.5;
        double minCircularity = 0;
        double maxCircularity = 1;
        double fx = CameraConstants.fx;
        double fy = CameraConstants.fy;
        double cx = CameraConstants.cx;
        double cy = CameraConstants.cy;
//        double [] relativeCameraPose = CameraConstants.relativeCameraPose;
//        double cameraPitch = CameraConstants.cameraPitch;
//        double ballPlaneHeight = CameraConstants.ballPlaneHeight;




        // If we want to run AprilTag detection on two portals simultaneously,
        // we need to create two distinct instances of the AprilTag processor,
        // one for each portal. If you want to see more detail about different
        // options that you have when creating these processors, go check out
        // the ConceptAprilTag OpMode.


        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(fx, fy, cx, cy)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        Scalar minValueY = new Scalar(0, 130, 40);
        Scalar maxValueY = new Scalar(255, 180, 110);
        ColorRange yellow = new ColorRange(ColorSpace.YCrCb, minValueY, maxValueY);

        Scalar minValueO = new Scalar(50, 150, 30);
        Scalar maxValueO = new Scalar(255, 200, 120);

        ColorRange orange = new ColorRange(ColorSpace.YCrCb, minValueO, maxValueO);


        colorLocatorProcessor =  new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(orange)   //ColorRange.YELLOW // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -0.5)) //Was .75 for each below, don't need to crop outer edges imo
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBlurSize(22)
                .setErodeSize(23)
                .setDilateSize(0)// Smooth the transitions between different colors in image
                .build();

        // Now we build both portals. The CRITICAL thing to notice here is the call to
        // setLiveViewContainerId(), where we pass in the IDs we received earlier from
        // makeMultiPortalView().
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280,800))
                //.enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(aprilTag)
                .addProcessor(colorLocatorProcessor)
                .build();
        boolean autoTuneMode = false;
        int innerIteration = 0;
        int outerIteration = 0;
        int [] gradientDirection = new int[]{5, 1};
        double gainLearningRate = 0.1;
        double exposureLearningRate = 0.07;
        double initalError = 0;
        double postGainError = 0;
        double postExposureError = 0;
        while (opModeInInit()) {



            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            exposure.setMode(ExposureControl.Mode.Manual);


            GainControl Gain = visionPortal.getCameraControl(GainControl.class);


            minExp = exposure.getMinExposure(TimeUnit.MILLISECONDS);
            maxExp = exposure.getMaxExposure(TimeUnit.MILLISECONDS);

            // Get webcam gain limits.
            minGain = Gain.getMinGain();
            maxGain = Gain.getMaxGain();

            if (exposureTime < maxExp && exposureTime > minExp) {
                exposure.setExposure(exposureTime, TimeUnit.MILLISECONDS);
            }
            if (gainNumber < maxGain && gainNumber > minGain) {
                Gain.setGain(gainNumber);
            }


            sleep(300);



            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocatorProcessor.getBlobs();

            //Blobs have to have a min contour area of 50 pixels and max of 20k pixels
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 750000, blobs);


            if (outerIteration > 25) {
                gainLearningRate = 0.05;
                exposureLearningRate = 0.035;
            }
            if (!(blobs.isEmpty())) {
                if (outerIteration < 50) {
                    int gainChange = gradientDirection[0];
                    int exposureChange = gradientDirection[1];
                    switch (innerIteration){
                        case 0:
                            initalError = BallAreaMethods.getLoss(blobs);
                            gainNumber+=gainChange;
                            innerIteration++;
                            break;
                        case 1:
                            postGainError = BallAreaMethods.getLoss(blobs);
                            gainNumber-=gainChange;
                            exposureTime+=exposureChange;
                            innerIteration++;
                            break;
                        case 2:
                            postExposureError = BallAreaMethods.getLoss(blobs);
                            exposureTime-=exposureChange;
                            innerIteration = 0;
                            outerIteration++;

                            double gainErrorChange = postGainError-initalError;
                            double exposureErrorChange = postExposureError-initalError;

                            int newGainChange = -1 * (int) Math.round(gainErrorChange * gainLearningRate);
                            if (newGainChange < -5) {
                                newGainChange = -5;
                            }
                            if (newGainChange > 5) {
                                newGainChange = 5;
                            }
                            int newExposureChange = -1 * (int) Math.round((exposureErrorChange * exposureLearningRate));
                            if (newExposureChange < -2) {
                                newExposureChange = -2;
                            }
                            if (newExposureChange > 2) {
                                newExposureChange = 2;
                            }
                            if (gainNumber + newGainChange < minGain) {
                                newGainChange = gainNumber - (minGain);
                            }
                            if (gainNumber + newGainChange > maxGain) {
                                newGainChange = maxGain - gainNumber;
                            }
                            if (exposureTime + newExposureChange < minExp) {
                                newExposureChange = exposureTime - (int) Math.round((minExp));
                            }
                            if (exposureTime + newExposureChange > maxExp) {
                                newExposureChange = (int) Math.round((maxExp)) - exposureTime;
                            }

                            if (newExposureChange == 0) {
                                newExposureChange = (exposureErrorChange < 0) ? 1 : -1;
                            }

                            if (newGainChange == 0) {
                                newGainChange = (gainErrorChange < 0) ? 1 : -1;
                            }

                            if (gainNumber >= maxGain) {
                                newGainChange = maxGain - gainNumber - 3;
                            }
                            if (gainNumber <= minGain) {
                                newGainChange = minGain - gainNumber + 3;
                            }
                            if (exposureTime >= maxExp) {
                                newExposureChange = (int) maxExp - exposureTime - 1;
                            }
                            if (exposureTime <= minExp) {
                                newExposureChange = (int) minExp - exposureTime + 1;
                            }


                            gradientDirection[0] = newGainChange;
                            gradientDirection[1] = newExposureChange;
                            break;


                    }



                }
            }




        }
        waitForStart();


        if (isStopRequested()) return;

        boolean started = false;
        boolean exposureMode = true;
        boolean triggeredBefore = false;


        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                if (!triggeredBefore) {
                    exposureMode = !exposureMode;
                }
                triggeredBefore = true;
            }
            else {
                triggeredBefore = false;
            }

            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(exposureTime, TimeUnit.MILLISECONDS);

            GainControl Gain = visionPortal.getCameraControl(GainControl.class);
            Gain.setGain(gainNumber);

            minExp = exposure.getMinExposure(TimeUnit.MILLISECONDS);
            maxExp = exposure.getMaxExposure(TimeUnit.MILLISECONDS);

            // Get webcam gain limits.
            minGain = Gain.getMinGain();
            maxGain = Gain.getMaxGain();

            if (gamepad1.dpad_right && circularityThreshold < maxCircularity) {
                circularityThreshold += 0.01;
            }
            if (gamepad1.dpad_left && circularityThreshold > minCircularity) {
                circularityThreshold -= 0.01;
            }


            if (exposureMode) {
                if (gamepad1.a && gainNumber < maxGain) {
                    gainNumber++;
                }
                else if (gamepad1.b && gainNumber > minGain) {
                    gainNumber--;
                }

                if (gamepad1.x && exposureTime < maxExp) {
                    exposureTime++;
                }
                else if (gamepad1.y && exposureTime > minExp) {
                    exposureTime--;
                }

                if (Gain.getGain() != gainNumber) {
                    Gain.setGain(gainNumber);
                }
                if (exposure.getExposure(TimeUnit.MILLISECONDS) != exposureTime) {
                    exposure.setExposure(exposureTime, TimeUnit.MILLISECONDS);
                }





            }


            if (frameNum == 500) {
                ActiveOpMode.telemetry().addLine("\nA: + Gain");
                ActiveOpMode.telemetry().addLine("B: - Gain");
                ActiveOpMode.telemetry().addLine("X: + Exposure time");
                ActiveOpMode.telemetry().addLine("Y: - Exposure time");

                ActiveOpMode.telemetry().addData("\nGain: ", gainNumber);
                ActiveOpMode.telemetry().addData("Exposure Time: ", exposureTime);
                ActiveOpMode.telemetry().addData("Circularity Threshold: ", circularityThreshold);
            }

//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            ActiveOpMode.telemetry().addData("# AprilTags Detected", currentDetections.size());

//            if (!currentDetections.isEmpty()) {
//                //I'm assuming the first tag it detects will be obelisk if multiple r in frame.
//                AprilTagDetection mainTag = currentDetections.get(0);
//                if (mainTag.metadata != null && mainTag.id != 20 && mainTag.id != 24) { //Change for biobuzz
//                    id = mainTag.id;
//                }
//                ActiveOpMode.telemetry().addData("April tag id: ", id);
//            }

            if (frameNum == 500) {
                List<ColorBlobLocatorProcessor.Blob> blobs = colorLocatorProcessor.getBlobs();

                //Blobs have to have a min contour area of 50 pixels and max of 20k pixels
                ColorBlobLocatorProcessor.Util.filterByCriteria(
                        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                        50, 750000, blobs);  // filter out very small blobs.

                //Blobs have to have a min aspect ratio of 1 (perfect square) and a max of 1.5 (one side is twice as long as its adjacent)
//                ColorBlobLocatorProcessor.Util.filterByCriteria(
//                        ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO,
//                        1, 1.4, blobs);


                int i = 0;

                for (ColorBlobLocatorProcessor.Blob b : blobs) {
                    if (b.getCircularity() > circularityThreshold) {
                        RotatedRect boxFit = b.getBoxFit();
                        ActiveOpMode.telemetry().addLine("------- Blob " + i + " -------");

                        double x = cx - (boxFit.center.x - cx); //THIS IS FLIPPED
                        double y = cy - (boxFit.center.y - cy);
                        ActiveOpMode.telemetry().addLine("Pixel Camera Coordinates: " + boxFit.center.x + ", " + boxFit.center.y);
                        double Xn = (x- cx) / fx;
                        double Yn = (y- cy) / fy;
                        double tx = Math.atan(Xn);
                        double ty = Math.atan(Yn);
                        ActiveOpMode.telemetry().addLine("Normalized Camera Coordinates: " + Xn + ", " + Yn);
                        ActiveOpMode.telemetry().addLine("Tx, Ty: " + tx + ", " + ty);

                        double[] relBallCoordinates = RayCastingMethods.getBallPose(x, y);
                        //Rounding coordinates to 100s place.
                        double ballX = (double) Math.round(relBallCoordinates[0] * 100) / 100;
                        double ballY = (double) Math.round(relBallCoordinates[1] * 100) / 100;
                        ActiveOpMode.telemetry().addLine("Ball Coordinates: " + ballX + ", " + ballY);

                        ActiveOpMode.telemetry().addLine("Contour Area: " + b.getContourArea() + ", Density: " + b.getDensity() +
                                ", Aspect Ratio: " + b.getAspectRatio() + ", Arc Length: " + (int) b.getArcLength() + ", Circularity: " + b.getCircularity());
                        //                telemetry.addLine(String.format("(%3d,%3d) %5d %4.2f  %5.2f %3d %5.3f ",
                        //                        (int) boxFit.center.x, (int) boxFit.center.y, b.getContourArea(), b.getDensity(),
                        //                        b.getAspectRatio(), (int) b.getArcLength(), b.getCircularity()));
                        i++;
                    }
                }
                frameNum = 0;
                ActiveOpMode.telemetry().update();
            }
            else {
                frameNum++;
            }

            if (gamepad1.right_trigger_pressed) {
                visionPortal.saveNextFrameRaw("Combined Test");
            }





        }

    }
}