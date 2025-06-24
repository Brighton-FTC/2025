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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

import kotlin.math.UMathKt;

@Config
@TeleOp(name = "Teleop (use this one)", group = "competition")
public class Teleop extends OpMode {
    public static double SLOW_MODE_SPEED = 0.6;


    private MecanumDrive drive;
    private GamepadEx gamepad1Ex, gamepad2Ex;

    private GrabberComponent grabber;

    private OpenCVComponent sensor;

    boolean cameraOn = false;

    private Motor horizontalSlideMotor;
//    private Motor verticalSlideMotor;

    private IntakeComponent intake;

    private HangComponent hang;
    private boolean isWinching = false;

    private LinearSlideComponent verticalSlide;

    private boolean isFieldCentric = true;

    private double inputMultiplier = 1;

    private IMU imu;

    public static final int HORIZONTAL_SLIDE_EXTENSION_LIMIT = -750;
    public static final int HORIZONTAL_SLIDE_RETRACT_LIMIT = 0;
    public static final int VERTICAL_SLIDE_UPPER_LIMIT = 2000;
    public static final int VERTICAL_SLIDE_LOWER_LIMIT = 250;

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


        sensor = new OpenCVComponent(hardwareMap, "Webcam 1", motors);


        horizontalSlideMotor = new Motor(hardwareMap, "horizontal_slide_motor");
        horizontalSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        horizontalSlideMotor.resetEncoder();

//        verticalSlideMotor = new Motor(hardwareMap, "vertical_slide_motor");
//        verticalSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        intake = new IntakeComponent(hardwareMap, "intake_motor");

        hang = new HangComponent(hardwareMap, "hang_motor", "hang_servo");




    }

    @Override
    public void loop() {
        gamepad1Ex.readButtons();
        gamepad2Ex.readButtons();

        if (cameraOn){sensor.isCentered();}


        // DRIVETRAIN

        double yaw = imu.getRobotYawPitchRollAngles().getYaw();

        if (gamepad1Ex.wasJustPressed(PSButtons.TRIANGLE)) {
            isFieldCentric = !isFieldCentric;
        }

        if (gamepad1Ex.wasJustPressed(PSButtons.CIRCLE)) {
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

        //OpenCV Sample Detection (red)
        if (gamepad1Ex.wasJustPressed(PSButtons.CROSS) && !cameraOn){
            sensor.switchToRed();
            cameraOn = true;
        }

        if (gamepad1Ex.wasJustPressed(PSButtons.SQUARE)&& !cameraOn){
            sensor.switchToBlue();
            cameraOn = true;
        }


        // VERTICAL SLIDE

//        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//            verticalSlide.up();
//        } else if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//            verticalSlide.down();
//        }
//
//        double rawInput = gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad2Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
//        if (verticalSlide.getMotor().getCurrentPosition() < VERTICAL_SLIDE_UPPER_LIMIT) {
//            verticalSlide.rawInput(-gamepad2Ex.getLeftY()); // negative goes up
//        } else {
//
//            verticalSlide.rawInput(Range.clip(-gamepad2Ex.getLeftY(), 0, 1.0)); // TODO: This assumes that negative motor power moves it DOWN. If that's wrong, CHANGE IT!!!
//        }

        //if (verticalSlide.getMotor().getCurrentPosition() > VERTICAL_SLIDE_UPPER_LIMIT) {
            //telemetry.addData("past limit", True);
            //verticalSlide.rawInput(Range.clip(gamepad2Ex.getLeftY(), 0, 1.0)); // TODO: This assumes that negative motor power moves it DOWN. If that's wrong, CHANGE IT!!!
        //} else if (verticalSlide.getMotor().getCurrentPosition() < VERTICAL_SLIDE_LOWER_LIMIT) {
            //verticalSlide.rawInput(Range.clip(gamepad2Ex.getLeftY(), -1, 0)); // negative goes up
//        if (verticalSlide.getMotor().getCurrentPosition() > VERTICAL_SLIDE_LOWER_LIMIT) {
//            verticalSlide.rawInput(-gamepad2Ex.getLeftY()); // negative goes up
//        }
//        else{
//            verticalSlide.rawInput(Range.clip(-gamepad2Ex.getLeftY(), -1, 0));
//        }
//        if (verticalSlide.getMotor().getCurrentPosition() < VERTICAL_SLIDE_UPPER_LIMIT && verticalSlide.getMotor().getCurrentPosition() > VERTICAL_SLIDE_LOWER_LIMIT){
//            verticalSlide.rawInput(-gamepad2Ex.getLeftY()); // negative goes up
//        //} else if (verticalSlide.getMotor().getCurrentPosition() < VERTICAL_SLIDE_UPPER_LIMIT) {
//            //verticalSlide.rawInput(Range.clip(-gamepad2Ex.getLeftY(), 0, 1.0)); // negative goes up
//        } else { // position is bigger than lower
//            //verticalSlide.rawInput(Range.clip(-gamepad2Ex.getLeftY(), -1, 0)); // negative goes up
//        }

        double P2LS = gamepad2Ex.getLeftY(); // negative goes up, positive goes down
        double deadZone = 0.2;

        verticalSlide.rawInput(0);

        // if negative input has large enough magnitude and slide is not lower than lower limit
        if (P2LS < -deadZone && verticalSlide.getMotor().getCurrentPosition() > VERTICAL_SLIDE_LOWER_LIMIT) {
            verticalSlide.rawInput(1); // go down
        } // if positive input has large enough magnitude and slide is not higher than upper limit
        else if (P2LS > deadZone && verticalSlide.getMotor().getCurrentPosition() < VERTICAL_SLIDE_UPPER_LIMIT) {
            verticalSlide.rawInput(-1); // up
        }



        verticalSlide.run();

        telemetry.addData("Vertical Slide Pos", verticalSlide.getMotor().getCurrentPosition());
        telemetry.addData("Vertical Slide Set-Point", verticalSlide.getSetPoint());
        telemetry.addLine();

        // CLAW

        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            grabber.toggleClaw();
        }

        telemetry.addLine(grabber.isClosed() ? "Grabber Closed" : "Grabber Open");
        telemetry.addLine();

////         vertical SLIDE
//        if (verticalSlideMotor.getCurrentPosition() < VERTICAL_SLIDE_UPPER_LIMIT) {
//            verticalSlideMotor.set(gamepad2Ex.getLeftY()); // if this is too fast, might add a x0.75 multiplier or something
//        } else {
//            verticalSlideMotor.set(-Math.abs(gamepad2Ex.getLeftY())); // TODO: This assumes that negative motor power moves it DOWN. If that's wrong, CHANGE IT!!!
//        }


        // horizontal slide
        // Check if above motor limit
        if (horizontalSlideMotor.getCurrentPosition() < HORIZONTAL_SLIDE_EXTENSION_LIMIT) {
            horizontalSlideMotor.set(Range.clip(gamepad2Ex.getRightY() * 0.5, -1.0, 0)); // joystick up, or positive is extension
        } else if (horizontalSlideMotor.getCurrentPosition() > HORIZONTAL_SLIDE_RETRACT_LIMIT) {
            horizontalSlideMotor.set(Range.clip(gamepad2Ex.getRightY() * 0.5, 0, 1)); // joystick up, or positive is extension
        } else {
            horizontalSlideMotor.set(gamepad2Ex.getRightY() * 0.5); // joystick up, or positive is extension
        }

        telemetry.addData("Horizontal Slide Pos", horizontalSlideMotor.getCurrentPosition());
        telemetry.addLine();

        // INTAKE

        // forward
        if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
            if (intake.getState() == IntakeComponent.State.FORWARD) {
                intake.setState(IntakeComponent.State.STOPPED);
            } else {
                intake.setState(IntakeComponent.State.FORWARD);
            }
        }

        // reverse
        if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
            if (intake.getState() == IntakeComponent.State.REVERSE) {
                intake.setState(IntakeComponent.State.STOPPED);
            } else {
                intake.setState(IntakeComponent.State.REVERSE);
            }
        }

        intake.run();

        telemetry.addData("Intake State", intake.getState().name());

//         HANG

        if (gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            hang.release();
        }

        if (gamepad1Ex.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            isWinching = !isWinching;
        }

        if (gamepad1Ex.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            hang.unwinch();
        }

        if (isWinching) {
            hang.winch();
        }

        if (gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            hang.returnToOriginal();
        }

        telemetry.addLine(hang.isReleased() ? "Hang released" : "Hang not released");
        telemetry.addLine(isWinching ? "Hang winching" : "Hang not winching");
        telemetry.addLine(hang.atSetPoint() ? "Hang completed" : "Hang not completed");
    }
}
