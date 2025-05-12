package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class OpenCVComponent {

    private final NormalizedColorSensor colorSensor;
    private final MecanumDrive drive;
    private final OpenCvWebcam webcam;
    private final BlueSamplePipeline bluePipeLine = new BlueSamplePipeline();
    private final RedSamplePipeline redPipeLine = new RedSamplePipeline();
    private final YellowSamplePipeline yellowPipeLine = new YellowSamplePipeline();

    MatOfPoint largestContour;
    boolean extending;

    public OpenCVComponent(HardwareMap hardwareMap, String sensorID, String webcamID) {

        Motor[] motors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        for (Motor motor : motors) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        motors[2].setInverted(true);
        // Initialize the color sensor
        colorSensor = (NormalizedColorSensor) hardwareMap.colorSensor.get(sensorID);
        colorSensor.setGain(2);  // Set the color sensor gain
        drive = new MecanumDrive(motors[0], motors[1], motors[2], motors[3]);
        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamID), cameraMonitorViewId);

    }

    public void startStreaming(){
        webcam.startStreaming(639, 479);
    }

    public void stopStreaming(){
        webcam.stopStreaming();
    }

    // Get the largest contour detected by the current pipeline
    // Switch to the blue pipeline
    public void switchToBlue() {
        webcam.setPipeline(bluePipeLine);
        largestContour = bluePipeLine.getLargestContour();
        if (largestContour != null) {
            Moments moments = Imgproc.moments(largestContour);
            double centerX = moments.get_m10() / moments.get_m00(); // X centroid of the contour
            while (centerX < 320) {
                drive.driveRobotCentric(0, -0.5, 0);  // Move forward
                moments = Imgproc.moments(largestContour);
                centerX = moments.get_m10() / moments.get_m00(); // Recalculate centroid
            }

            drive.driveRobotCentric(0, 0, 0);
        }
    }

    // Switch to the red pipeline
    public void switchToRed() {
        webcam.setPipeline(redPipeLine);
        largestContour = redPipeLine.getLargestContour();
        if (largestContour != null) {
            Moments moments = Imgproc.moments(largestContour);
            double centerX = moments.get_m10() / moments.get_m00(); // X centroid of the contour
            while (centerX < 320 ) {
                drive.driveRobotCentric(0, -0.5, 0);  // Move forward
                moments = Imgproc.moments(largestContour);
                centerX = moments.get_m10() / moments.get_m00(); // Recalculate centroid
            }

            drive.driveRobotCentric(0, 0, 0);
        }
    }

    // Switch to the yellow pipeline
    public void switchToYellow() {
        webcam.setPipeline(yellowPipeLine);
        largestContour = yellowPipeLine.getLargestContour();

        if (largestContour != null) {
            Moments moments = Imgproc.moments(largestContour);
            double centerX = moments.get_m10() / moments.get_m00(); // X centroid of the contour
            while (centerX < 320) {
                drive.driveRobotCentric(0, -0.5, 0);  // Move forward
                moments = Imgproc.moments(largestContour);
                centerX = moments.get_m10() / moments.get_m00(); // Recalculate centroid
            }

            drive.driveRobotCentric(0, 0, 0);
        }
    }

    public void blueCheck(){
        extending = true;
        while (extending) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            double blue = colors.blue;
            if (blue>0.5) {
                extending = false;
            }

        }

    }

    public void redCheck(){
        extending = true;
        while (extending) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            double red = colors.red;
            if (red>0.5) {
                extending = false;
            }

        }

    }


}
