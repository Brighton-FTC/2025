package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class OpenCVComponent {

    private final OpenCvWebcam webcam;
    private final BlueSamplePipeline bluePipeLine = new BlueSamplePipeline();
    private final RedSamplePipeline redPipeLine = new RedSamplePipeline();
    private final YellowSamplePipeline yellowPipeLine = new YellowSamplePipeline();

    MatOfPoint largestContour;
    boolean extending;

    public OpenCVComponent(HardwareMap hardwareMap, String webcamID) {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamID), cameraMonitorViewId);


    }

    public void startStreaming() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", errorCode);
                telemetry.update();
            }
        });
    }

    public void stopStreaming() {
        webcam.stopStreaming();
    }

    // Get the largest contour detected by the current pipeline
    // Switch to the blue pipeline
    public void switchToBlue() {
        startStreaming();
        webcam.setPipeline(bluePipeLine);
        largestContour = bluePipeLine.getLargestContour();
        if (largestContour != null) {
            Moments moments = Imgproc.moments(largestContour);
            double centerX = moments.get_m10() / moments.get_m00(); // X centroid of the contour
            while (Math.abs(centerX - 160) > 5) {
                telemetry.addData("Centered", false);
                telemetry.update();

            }
            telemetry.addData("Centered", true);
            telemetry.update();


        }
    }

    // Switch to the red pipeline
    public void switchToRed() {
        startStreaming();
        webcam.setPipeline(redPipeLine);
        largestContour = redPipeLine.getLargestContour();
        if (largestContour != null) {
            Moments moments = Imgproc.moments(largestContour);
            double centerX = moments.get_m10() / moments.get_m00(); // X centroid of the contour
            while (Math.abs(centerX - 160) > 5) {
                moments = Imgproc.moments(largestContour);
                centerX = moments.get_m10() / moments.get_m00();
                telemetry.addData("Centered", false);
                telemetry.update();// Recalculate centroid
            }
            telemetry.addData("Centered", true);
            telemetry.update();

        }
    }

    // Switch to the yellow pipeline
    public void switchToYellow() {
        startStreaming();
        webcam.setPipeline(yellowPipeLine);
        largestContour = yellowPipeLine.getLargestContour();

        if (largestContour != null) {
            Moments moments = Imgproc.moments(largestContour);
            double centerX = moments.get_m10() / moments.get_m00(); // X centroid of the contour
            while (Math.abs(centerX - 160) > 5) {
                moments = Imgproc.moments(largestContour);
                centerX = moments.get_m10() / moments.get_m00();
                telemetry.addData("Centered", false);// Recalculate centroid
                telemetry.update();
            }
            telemetry.addData("Centered", true);
            telemetry.update();

        }
    }


}