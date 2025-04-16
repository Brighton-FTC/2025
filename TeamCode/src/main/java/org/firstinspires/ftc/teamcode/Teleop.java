package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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
    private GamepadEx gamepad;
    private IMU imu;

    private boolean isFieldCentric = true;

    private double inputMultiplier = 1;

    public static double kP = -0.015, kI = 0, kD = 0;
    public static double HEADING_TOLERANCE = 3;
    private final PIDController headingController = new PIDController(kP, kI, kD);

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        gamepad = gamepad1Ex;

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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        imu.resetYaw();
    }

    @Override
    public void loop() {
        // update values every tick, for tuning
        headingController.setPID(kP, kI, kD);
        headingController.setTolerance(HEADING_TOLERANCE);

        gamepad1Ex.readButtons();
        gamepad2Ex.readButtons();

        double yaw = imu.getRobotYawPitchRollAngles().getYaw();

        if (gamepad.wasJustPressed(PSButtons.TRIANGLE)) {
            isFieldCentric = !isFieldCentric;
        }

        if (gamepad.wasJustPressed(PSButtons.CROSS)) {
            inputMultiplier = inputMultiplier == 1 ? SLOW_MODE_SPEED : 1;
        }

        if (gamepad.getRightX() != 0) {
            if (isFieldCentric) {
                drive.driveFieldCentric(gamepad.getLeftX() * inputMultiplier,
                        gamepad.getLeftY() * inputMultiplier,
                        gamepad.getRightX() * inputMultiplier,
                        yaw, true);
            } else {
                drive.driveRobotCentric(gamepad.getLeftX() * inputMultiplier,
                        gamepad.getLeftY() * inputMultiplier,
                        gamepad.getRightX() * inputMultiplier,
                        true);
            }

            headingController.setSetPoint(yaw);

        } else {
            if (isFieldCentric) {
                drive.driveFieldCentric(gamepad.getLeftX() * inputMultiplier,
                        gamepad.getLeftY() * inputMultiplier,
                        headingController.atSetPoint() ? 0 : headingController.calculate(correctYaw(yaw, headingController.getSetPoint())),
                        yaw, true);
            } else {
                drive.driveRobotCentric(gamepad.getLeftX() * inputMultiplier,
                        gamepad.getLeftY() * inputMultiplier,
                        headingController.atSetPoint() ?  0 : headingController.calculate(correctYaw(yaw, headingController.getSetPoint())),
                        true);
            }
        }

        telemetry.addData("Control", gamepad == gamepad1Ex ? "Gamepad 1" : "Gamepad 2");
        telemetry.addLine();

        telemetry.addLine(isFieldCentric ? "Driving Field Centric" : "Driving Robot Centric");
        if (inputMultiplier == SLOW_MODE_SPEED) {
            telemetry.addLine("Slow Mode Activated");
        }
        telemetry.addData("Heading", yaw);
        telemetry.addData("Target Heading", headingController.getSetPoint());
        telemetry.addLine();
    }

    // wraps yaw, in degrees
    private double correctYaw(double yaw, double expectedYaw) {
        while (expectedYaw - yaw > 180) {
            yaw += 360;
        }

        while (expectedYaw - yaw < -180) {
            yaw -= 360;
        }

        return yaw;
    }
}