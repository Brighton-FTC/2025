package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
public class ColorSensorComponent {
    private final NormalizedColorSensor colorSensor;

    String targetColor;

    double BLUE_THRESHOLD = 0.05;
    double RED_THRESHOLD = 0.05;

    public ColorSensorComponent(HardwareMap hardwareMap, String SensorID) {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, SensorID);
        colorSensor.setGain(2);
    }

    public void runComponent(){
        telemetry.addData("Raw Red", colorSensor.getNormalizedColors().red);
        telemetry.addData("Raw Blue", colorSensor.getNormalizedColors().blue);
        if ("red".equals(targetColor)){
            telemetry.addData("Red Detected", redDetected());
            telemetry.update();
        } else if ("blue".equals(targetColor)) {
            telemetry.addData("Blue Detected", blueDetected());
            telemetry.update();
        }
        else {
            telemetry.addData("color detected", false);
            telemetry.update();
        }

    }

    public boolean blueDetected(){
        return colorSensor.getNormalizedColors().blue > BLUE_THRESHOLD;

    }

    public boolean redDetected(){
        return colorSensor.getNormalizedColors().red > RED_THRESHOLD;

    }

    public void switchToRed(){
        targetColor = "red";
    }

    public void switchToBlue(){
        targetColor = "blue";
    }



}
