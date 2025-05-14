package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp
class ColorSensorTester extends OpMode {
    private ColorSensorComponent sensor;


    GamepadEx gamePad;

    @Override
    public void init() {
        sensor = new ColorSensorComponent(hardwareMap, "color-sensor");
        gamePad = new GamepadEx(gamepad1);
    }


    @Override
    public void loop() {


        if (gamePad.wasJustPressed(PSButtons.SQUARE)) {
            sensor.blueCheck();
            sensor.runComponent();

        }

        if (gamePad.wasJustPressed(PSButtons.CIRCLE)){
            sensor.redCheck();
            sensor.runComponent();

        }

    }



    }

