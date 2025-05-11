package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Config
public class ColorSensorComponent {
    private final NormalizedColorSensor colorSensor;
    boolean moving;

    double travelDistance;

    double currentDistance;

    private final MecanumDrive drive;

    double FinalEX;

    public ColorSensorComponent(HardwareMap hardwareMap, String SensorID, MecanumDrive drive) {
        colorSensor = (NormalizedColorSensor) hardwareMap.colorSensor.get(SensorID);
        colorSensor.setGain(2);
        this.drive = drive;
    }

    public void runComponent(){
        while(moving){
            for (double CurrentEX = 0; CurrentEX <= FinalEX;) {
                while(currentDistance<=travelDistance) {
                    drive.driveRobotCentric(0, -2, 0);
                    currentDistance+=2;
                }
                while(currentDistance<=travelDistance) {
                    drive.driveRobotCentric(0, 2, 0);
                    currentDistance+=2;
                }
            }
            moving = false;
        }
    }

    public void blueCheck(){
        moving = true;
        while (moving) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            double blue = colors.blue;


            if (blue > 0.5) {
                moving = false;
            }

        }

    }

    public void redCheck(){
        moving = true;
        while (moving) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            double red = colors.red;


            if (red > 0.5) {
                moving = false;
            }

        }

    }


}
