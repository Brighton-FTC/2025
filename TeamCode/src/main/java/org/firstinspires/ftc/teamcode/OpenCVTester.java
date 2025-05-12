package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp
public class OpenCVTester extends OpMode {

    private OpenCVComponent sensor;

    private MecanumDrive drive;

    GamepadEx gamePad;

    @Override
    public void init() {
        sensor = new OpenCVComponent(hardwareMap, "color-sensor", "Webcam 1");
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
