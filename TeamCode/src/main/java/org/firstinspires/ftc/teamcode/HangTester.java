package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@Config
@TeleOp
public class HangTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HangComponent hang = new HangComponent(hardwareMap, "hang_motor", "hang_servo");
        GamepadEx gamepad = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Motor hori_motor = new Motor(hardwareMap, "vertical_slide_motor");

        waitForStart();

        boolean isWinching = false;

        while (!isStopRequested()) {
            gamepad.readButtons();

            if (gamepad.wasJustPressed(PSButtons.CIRCLE)) {
                hang.release();
            }

            if (gamepad.wasJustPressed(PSButtons.SQUARE)) {
                hang.unwinch();
            }

            if (gamepad.wasJustPressed(PSButtons.TRIANGLE)) {
                isWinching = !isWinching;
            }


            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                hang.returnToOriginal();
            }

            if (isWinching) {
                hang.winch();
                telemetry.addLine(hang.isReleased() ? "Hang released" : "Hang not released");
            }

            telemetry.addLine(hang.isReleased() ? "Hang released" : "Hang not released");
            telemetry.addLine(isWinching ? "Hang winching" : "Hang not winching");
            telemetry.addLine(hang.atSetPoint() ? "Hang completed" : "Hang not completed");
            telemetry.addData("Hang motor pos ", hang.getMotorPos());
            telemetry.addData("hor motor ", hori_motor.getCurrentPosition());

            telemetry.update();
            sleep(20);
        }
    }
}
