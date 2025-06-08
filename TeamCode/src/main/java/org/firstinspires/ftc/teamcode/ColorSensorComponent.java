package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Config
public class ColorSensorComponent {
    private final NormalizedColorSensor colorSensor;
    boolean moving;

    double travelDistance;

    double currentDistance;


    double FinalEX;

    public ColorSensorComponent(HardwareMap hardwareMap, String SensorID) {


        colorSensor = hardwareMap.get(NormalizedColorSensor.class, SensorID);
        colorSensor.setGain(2);
    }

    public void runComponent(){
        while(moving){
            for (double CurrentEX = 0; CurrentEX <= FinalEX;) {
                while(currentDistance<=travelDistance) {
                    currentDistance+=2;
                }
                while(currentDistance<=travelDistance) {
                    currentDistance+=2;
                }
            }
            moving = false;
        }
    }

    public boolean blueDetected(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double blue = colors.blue;


        return blue > 0.5;

    }

    public boolean redDetected(){

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double red = colors.red;



        return red > 0.5;

    }

    public void checkRed(){
        telemetry.addData("Red Detected:", redDetected());
    }

    public void checkBlue(){
        telemetry.addData("Blue Detected:", blueDetected());
    }


}
