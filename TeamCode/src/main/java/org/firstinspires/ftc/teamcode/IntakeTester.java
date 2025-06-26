package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

public class IntakeTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IntakeComponent intake = new IntakeComponent(hardwareMap, "intake_motor", "intake_servo");
        GamepadEx gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            gamepad.readButtons();

            // forward
            if (gamepad.wasJustPressed(PSButtons.TRIANGLE)) {
                if (intake.getState() == IntakeComponent.State.FORWARD) {
                    intake.setState(IntakeComponent.State.STOPPED);
                } else {
                    intake.setState(IntakeComponent.State.FORWARD);
                }
            }

            // reverse
            if (gamepad.wasJustPressed(PSButtons.CIRCLE)) {
                if (intake.getState() == IntakeComponent.State.REVERSE) {
                    intake.setState(IntakeComponent.State.STOPPED);
                } else {
                    intake.setState(IntakeComponent.State.REVERSE);
                }
            }

            intake.run();

            telemetry.addData("Intake State", intake.getState().name());
            telemetry.update();
            sleep(20);
        }
    }
}
