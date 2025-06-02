package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp
public class ColorSensorTest extends OpMode {
    private ColorSensorComponent sensor;


    GamepadEx gamePad;

    @Override
    public void init() {

        sensor = new ColorSensorComponent(hardwareMap, "sensor_color");
        gamePad = new GamepadEx(gamepad1);
    }


    @Override
    public void loop() {


        if (gamePad.wasJustPressed(PSButtons.SQUARE)) {
            telemetry.addData("Blue Detected:", sensor.blueDetected());
        }

        if (gamePad.wasJustPressed(PSButtons.CIRCLE)){
            telemetry.addData("Red Detected:", sensor.redDetected());

        }

    }



    }

