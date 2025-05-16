package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp
public class OpenCVTester extends OpMode {

    private OpenCVComponent sensor;

    GamepadEx gamePad;

    @Override
    public void init() {

        Motor[] motors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        sensor = new OpenCVComponent(hardwareMap, "sensor_color", "Webcam 1", motors);
        gamePad = new GamepadEx(gamepad1);



    }


    @Override
    public void loop() {


        if (gamePad.wasJustPressed(PSButtons.SQUARE)) {
            sensor.switchToBlue();
            sensor.blueCheck();
        }

        if (gamePad.wasJustPressed(PSButtons.CIRCLE)){
            sensor.switchToRed();
            sensor.redCheck();

        }

    }
}
