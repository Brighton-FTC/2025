package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
public class PipLineComponent {

    private final NormalizedColorSensor colorSensor;
    private MecanumDrive drive;
    double currentDistance;

    boolean extending;

    public PipLineComponent(HardwareMap hardwareMap, String SensorID, MecanumDrive drive) {
        colorSensor = (NormalizedColorSensor) hardwareMap.colorSensor.get(SensorID);
        colorSensor.setGain(2);
        this.drive = drive;

    }
    public void runOpenCV(double posX){
        while(currentDistance<=posX) {
            drive.driveRobotCentric(-2, 0, 0);
            currentDistance+=2;
        }


        //still need to add slide.
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
