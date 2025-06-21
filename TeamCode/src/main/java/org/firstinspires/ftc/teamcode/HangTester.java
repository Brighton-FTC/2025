package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

public class HangTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HangComponent hang = new HangComponent(hardwareMap, "hang_motor", "hang_servo");
        GamepadEx gamepad = new GamepadEx(gamepad1);

        waitForStart();

        boolean isWinching = false;

        while (!isStopRequested()) {
            gamepad.readButtons();

            if (gamepad.wasJustPressed(PSButtons.CIRCLE)) {
                hang.release();
            }

            if (gamepad.wasJustPressed(PSButtons.TRIANGLE)) {
                isWinching = !isWinching;
            }

            if (isWinching) {
                hang.winch();
            }

            telemetry.addLine(hang.isReleased() ? "Hang released" : "Hang not released");
            telemetry.addLine(isWinching ? "Hang winching" : "Hang not winching");
            telemetry.addLine(hang.atSetPoint() ? "Hang completed" : "Hang not completed");

            telemetry.update();
            sleep(20);
        }
    }
}
