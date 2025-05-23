package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class OpenCVComponent {

    private final OpenCvWebcam webcam;
    private final BlueSamplePipeline bluePipeLine = new BlueSamplePipeline();
    private final RedSamplePipeline redPipeLine = new RedSamplePipeline();
    private final YellowSamplePipeline yellowPipeLine = new YellowSamplePipeline();

    private SamplePipeline currentPipeLine;

    private MecanumDrive drive;

    MatOfPoint largestContour;
    boolean extending;

    public OpenCVComponent(HardwareMap hardwareMap, String webcamID, Motor[] motors) {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamID), cameraMonitorViewId);


        motors[2].setInverted(true);

        drive = new MecanumDrive(motors[0], motors[1], motors[2], motors[3]);





    }

    public void startStreaming() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 60);


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
        telemetry.addData("Camera", "On");
        webcam.setPipeline(bluePipeLine);
        currentPipeLine = bluePipeLine;
        telemetry.update();
    }

    public void switchToRed() {
        startStreaming();
        telemetry.addData("Camera", "On");
        webcam.setPipeline(redPipeLine);
        currentPipeLine = redPipeLine;
        telemetry.update();
    }

    public void isCentered() {
        MatOfPoint contour = null;
        if (currentPipeLine == bluePipeLine) {
            contour = bluePipeLine.getLargestContour();
        } else if (currentPipeLine == redPipeLine) {
            contour = redPipeLine.getLargestContour();
        } else if (currentPipeLine == yellowPipeLine) {
            contour = yellowPipeLine.getLargestContour();
        }

        if (contour == null) return;

        Moments moments = Imgproc.moments(contour);
        if (moments.get_m00() == 0) return;

        if (largestContour != null) {
            telemetry.addData("Contour Area", Imgproc.contourArea(largestContour));
        } else {
            telemetry.addData("Contour", "None");
        }
        telemetry.update();

        // Avoid division by zero

        double centerX = moments.get_m10() / moments.get_m00();
        if (Math.abs(centerX - 320) >= 60){
            drive.driveRobotCentric(0, -(centerX-320), 0);
        }
    }


    // Switch to the yellow pipeline
    public void switchToYellow() {
        startStreaming();
        telemetry.addData("Camera", "On");
        webcam.setPipeline(bluePipeLine);
        currentPipeLine = bluePipeLine;
        telemetry.update();
    }





}