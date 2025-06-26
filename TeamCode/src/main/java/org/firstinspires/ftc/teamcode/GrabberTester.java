package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp
public class GrabberTester extends OpMode {
    private GrabberComponent grabber;

    private GamepadEx gamepad;

    @Override
    public void init() {
        grabber = new GrabberComponent(hardwareMap, "claw_servo");
        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.readButtons();

        if (gamepad.wasJustPressed(PSButtons.SQUARE)) {
            grabber.reset(); //open
        }

        if (gamepad.wasJustPressed(PSButtons.CROSS)) {
            grabber.grab(); //close
        }

        if (gamepad.wasJustPressed(PSButtons.CIRCLE)) {
            grabber.toggleClaw();
        }



        telemetry.addData("Grabber Status", grabber.isClosed() ? "Closed" : "Opened");
    }
}
