package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp(name = "Sample Color Sensing", group = "Sensor")
public class SampleColorSensor extends OpMode {

    NormalizedColorSensor colorSensor;

    GamepadEx gamepad;
    boolean moving = false;

    @Override
    public void init(){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        gamepad = new GamepadEx(gamepad1);

        // Optional: Set an initial gain for lighting conditions
        colorSensor.setGain(2);

    }






    @Override
    public void loop() {


        if (gamepad.getButton(PSButtons.SQUARE)){
            moving = true;
            while (moving) {
                NormalizedRGBA colors = colorSensor.getNormalizedColors();

                double blue = colors.blue;


                if (blue > 0.5) {
                    moving = false;
                }

            }

        }


        if (gamepad.getButton(PSButtons.CIRCLE)){
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
}
