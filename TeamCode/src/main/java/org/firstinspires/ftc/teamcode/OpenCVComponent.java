package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;





@TeleOp
public class OpenCVComponent {

    private final NormalizedColorSensor colorSensor;
    private final MecanumDrive drive;
    private final OpenCvWebcam webcam;
    private final BlueSamplePipeline bluePipeLine = new BlueSamplePipeline();
    private final RedSamplePipeline redPipeLine = new RedSamplePipeline();
    private final YellowSamplePipeline yellowPipeLine = new YellowSamplePipeline();

    private OpenCvPipeline currentPipeline;
    double currentDistance;
    MatOfPoint largestContour;
    boolean extending;

    public OpenCVComponent(HardwareMap hardwareMap, String sensorID, MecanumDrive drive, String webcamID) {
        // Initialize the color sensor
        colorSensor = (NormalizedColorSensor) hardwareMap.colorSensor.get(sensorID);
        colorSensor.setGain(2);  // Set the color sensor gain
        this.drive = drive;

        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamID), cameraMonitorViewId);

    }


    public void startStreaming(){
        webcam.startStreaming(320, 240);
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
            while (centerX > 0) {
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
            while (centerX > 0) {
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
            while (centerX > 0) {
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
