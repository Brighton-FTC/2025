package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp
class ColorSensorTester extends OpMode {
    private ColorSensorComponent sensor;

    private MecanumDrive drive;
    private IMU imu;

    GamepadEx gamePad;

    @Override
    public void init() {
        sensor = new ColorSensorComponent(hardwareMap, "color-sensor");
        gamePad = new GamepadEx(gamepad1);

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


        if (gamePad.wasJustPressed(PSButtons.SQUARE)) {
            sensor.blueCheck();
            sensor.runComponent();

        }

        if (gamePad.wasJustPressed(PSButtons.CIRCLE)){
            sensor.redCheck();
            sensor.runComponent();

        }

    }



    }

