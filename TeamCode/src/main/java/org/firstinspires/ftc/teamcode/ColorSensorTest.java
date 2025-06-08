package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp
public class ColorSensorTest extends OpMode {
    private ColorSensorComponent sensor;


    GamepadEx gamePad;

    @Override
    public void init() {
        sensor = new ColorSensorComponent(hardwareMap, "sensor_color");
        gamePad = new GamepadEx(gamepad1);
        telemetry.addData("Initialised", "true");
    }


    @Override
    public void loop() {
        gamePad.readButtons();

        if (gamePad.wasJustPressed(PSButtons.SQUARE)) {
            sensor.checkRed();
        }

        if (gamePad.wasJustPressed(PSButtons.CIRCLE)){
            sensor.checkBlue();

        }

    }



    }

