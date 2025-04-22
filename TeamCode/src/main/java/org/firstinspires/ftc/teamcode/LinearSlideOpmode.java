package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class LinearSlideOpmode extends OpMode {

    private GamepadEx gamepad;
    private LinearSlideComponent linearSlide;

    private boolean isRunningArm = true;

    @Override
    public void init() {
        linearSlide = new LinearSlideComponent(hardwareMap, "linear_slide_motor", "arm_sensor");
        gamepad = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        gamepad.readButtons();

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            linearSlide.up();
            isRunningArm = true;
        } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            linearSlide.down();
            isRunningArm = true;

        } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            linearSlide.downToSwitch();
            isRunningArm = true;

        } else if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0) {
            isRunningArm = false;
        }

        if (isRunningArm) {
            linearSlide.run();
        } else {
            linearSlide.getMotor().set(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                    - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        }

        telemetry.addData("Position", linearSlide.getMotor().getCurrentPosition());
        telemetry.addData("Set point", linearSlide.getSetPoint());
        telemetry.addData("At Set-Point", linearSlide.atSetPoint());
    }

}
