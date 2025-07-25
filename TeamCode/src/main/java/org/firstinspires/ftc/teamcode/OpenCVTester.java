package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp
public class OpenCVTester extends OpMode {

    private OpenCVComponent sensor;

    GamepadEx gamePad;

    boolean cameraOn = false;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Init started");
        telemetry.update();

        Motor[] motors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        for (Motor motor : motors) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }


        try {
            sensor = new OpenCVComponent(hardwareMap, "Webcam 1", motors);
            telemetry.addData("OpenCV", "Initialized webcam");
        } catch (Exception e) {
            telemetry.addData("OpenCV ERROR", e.toString());
            telemetry.update();
            return;
        }

        try {
            gamePad = new GamepadEx(gamepad1);
            telemetry.addData("Gamepad", "GamepadEx initialized");
        } catch (Exception e) {
            telemetry.addData("Gamepad ERROR", e.toString());
            telemetry.update();
            return;
        }

        telemetry.addData("Status", "Init complete");
        telemetry.update();




    }



    @Override
    public void loop() {
        telemetry.addData("Loop", "Running"); // <--- NEW: confirms loop is executing
        gamePad.readButtons(); // <--- Mandatory!

        telemetry.addData("SQUARE", gamePad.wasJustPressed(PSButtons.SQUARE));
        telemetry.addData("CIRCLE", gamePad.wasJustPressed(PSButtons.CIRCLE));
        telemetry.addData("CameraOn", cameraOn);
        sensor.isCentered();

        telemetry.update();

        // Handle gamepad button presses
        if (gamePad.wasJustPressed(PSButtons.SQUARE)&&!cameraOn) {
            sensor.switchToBlue();
            cameraOn = true;
        } else if (gamePad.wasJustPressed(PSButtons.CIRCLE)&&!cameraOn) {
            sensor.switchToRed();
            cameraOn = true;
        }else if(gamePad.wasJustPressed(PSButtons.CROSS)&&!cameraOn){
            sensor.switchToYellow();
            cameraOn = true;
        }
        else if (gamePad.wasJustPressed(PSButtons.CIRCLE) || gamePad.wasJustPressed(PSButtons.SQUARE) && cameraOn){
            cameraOn = false;
            sensor.stopStreaming();
        }
    }


}
