package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@Config
@TeleOp(name = "Teleop (use this one)", group = "competition")
public class Teleop extends OpMode {
    public static double SLOW_MODE_SPEED = 0.6;


    private MecanumDrive drive;
    private GamepadEx gamepad1Ex, gamepad2Ex;

    private GrabberComponent grabber;
    private LinearSlideComponent verticalSlide;

    private Motor horizontalSlideMotor;

    private IntakeComponent intake;

    private HangComponent hang;
    private boolean isWinching = false;

    private boolean isFieldCentric = true;

    private double inputMultiplier = 1;

    private IMU imu;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        Motor[] motors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        for (Motor motor : motors) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        motors[2].setInverted(true);

        drive = new MecanumDrive(motors[0], motors[1], motors[2], motors[3]);

        verticalSlide = new LinearSlideComponent(hardwareMap, "vertical_slide_motor", "vertical_slide_sensor");
        grabber = new GrabberComponent(hardwareMap, "claw_servo");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT // TODO: can't remember whether this was left or right, so change if necessary
                )
        ));

        horizontalSlideMotor = new Motor(hardwareMap, "horizontal_slide_motor");
        horizontalSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        intake = new IntakeComponent(hardwareMap, "intake_motor");

        hang = new HangComponent(hardwareMap, "hang_motor", "hang_servo");
    }

    @Override
    public void loop() {
        gamepad1Ex.readButtons();
        gamepad2Ex.readButtons();

        // DRIVETRAIN

        double yaw = imu.getRobotYawPitchRollAngles().getYaw();

        if (gamepad1Ex.wasJustPressed(PSButtons.TRIANGLE)) {
            isFieldCentric = !isFieldCentric;
        }

        if (gamepad1Ex.wasJustPressed(PSButtons.CROSS)) {
            inputMultiplier = inputMultiplier == 1 ? SLOW_MODE_SPEED : 1;
        }

        if (isFieldCentric) {
            drive.driveFieldCentric(gamepad1Ex.getLeftX() * inputMultiplier,
                    gamepad1Ex.getLeftY() * inputMultiplier,
                    gamepad1Ex.getRightX() * inputMultiplier,
                    yaw, true);
        } else {
            drive.driveRobotCentric(gamepad1Ex.getLeftX() * inputMultiplier,
                    gamepad1Ex.getLeftY() * inputMultiplier,
                    gamepad1Ex.getRightX() * inputMultiplier,
                    true);
        }

        telemetry.addLine(isFieldCentric ? "Driving Field Centric" : "Driving Robot Centric");
        if (inputMultiplier == SLOW_MODE_SPEED) {
            telemetry.addLine("Slow Mode Activated");
        }
        telemetry.addData("Heading", yaw);
        telemetry.addLine();

        // VERTICAL SLIDE

        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            verticalSlide.up();
        } else if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            verticalSlide.run();
        }

        double rawInput = gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad2Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        if (rawInput != 0) {
            verticalSlide.rawInput(rawInput);
        }

        verticalSlide.run();

        telemetry.addData("Vertical Slide Pos", verticalSlide.getMotor().getCurrentPosition());
        telemetry.addData("Vertical Slide Set-Point", verticalSlide.getSetPoint());
        telemetry.addLine();

        // CLAW

        if (gamepad2Ex.wasJustPressed(PSButtons.SQUARE)) {
            grabber.toggleClaw();
        }

        telemetry.addLine(grabber.isClosed() ? "Grabber Closed" : "Grabber Open");
        telemetry.addLine();

        // HORIZONTAL SLIDE
        horizontalSlideMotor.set(gamepad2Ex.getLeftY()); // if this is too fast, might add a x0.75 multiplier or something

        // INTAKE

        // forward
        if (gamepad2Ex.wasJustPressed(PSButtons.TRIANGLE)) {
            if (intake.getState() == IntakeComponent.State.FORWARD) {
                intake.setState(IntakeComponent.State.STOPPED);
            } else {
                intake.setState(IntakeComponent.State.FORWARD);
            }
        }

        // reverse
        if (gamepad2Ex.wasJustPressed(PSButtons.CIRCLE)) {
            if (intake.getState() == IntakeComponent.State.REVERSE) {
                intake.setState(IntakeComponent.State.STOPPED);
            } else {
                intake.setState(IntakeComponent.State.REVERSE);
            }
        }

        intake.run();

        telemetry.addData("Intake State", intake.getState().name());

        // HANG

        if (gamepad1Ex.wasJustPressed(PSButtons.CIRCLE)) {
            hang.release();
        }

        if (gamepad1Ex.wasJustPressed(PSButtons.TRIANGLE)) {
            isWinching = !isWinching;
        }

        if (isWinching) {
            hang.winch();
        }

        telemetry.addLine(hang.isReleased() ? "Hang released" : "Hang not released");
        telemetry.addLine(isWinching ? "Hang winching" : "Hang not winching");
        telemetry.addLine(hang.atSetPoint() ? "Hang completed" : "Hang not completed");
    }
}
