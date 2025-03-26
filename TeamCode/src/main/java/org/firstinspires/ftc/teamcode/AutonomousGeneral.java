package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;


@Config
@Autonomous(name = "General Autonomous", preselectTeleOp = "Teleop")
public class AutonomousGeneral extends LinearOpMode {
    protected MecanumDrive drive;


    public static double startX = 0;
    public static double startY = 0;
    public static double startHeading = Math.toRadians(0);

    public static double basket1X = 3;
    public static double basket1Y = 21;
    public static double basket1Tangent = Math.toRadians(180);
    public static double basket1Heading = Math.toRadians(-157.5);

    public static double spike1X = 36;
    public static double spike1Y = 12;
    public static double spike1Tangent = Math.toRadians(-45);

    // at the moment, error in the robot's odometry builds up over time and causes it to be a few inches off the second time it goes to the basket
    // ideally would like to fix this, but this works for now
    public static double basket2X = 3;
    public static double basket2Y = 26;
    public static double basket2Tangent = Math.toRadians(180);
    public static double basket2Heading = Math.toRadians(-157.5);

    public static double spike2X = 38;
    public static double spike2Y = 30;
    public static double spike2Tangent = Math.toRadians(-45);

    public static double parkX = 54;
    public static double parkY = 3;
    public static double parkTangent = Math.toRadians(-90);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "linear_slide_motor");

        GrabberComponent grabber = new GrabberComponent(hardwareMap, "left_claw_servo", "right_claw_servo", "rotator_servo");
        grabber.grab();

        Action startToBasket = drive.actionBuilder(new Pose2d(startX, startY, startHeading))
                .afterDisp(0, linearSlide::up)
//                .splineTo(new Vector2d(12, basket1Y / 2), Math.toRadians(90))
//                .waitSeconds(WAIT_TIME)
                .splineTo(new Vector2d(basket1X, basket1Y), basket1Tangent)
                .turnTo(basket1Heading)
                .build();

        Action basketToSpike1 = drive.actionBuilder(new Pose2d(basket1X, basket1Y, basket1Heading))
                //.setTangent(Math.toRadians(basketToSpikeHeading))
                .splineToSplineHeading(new Pose2d(spike1X, spike1Y, spike1Tangent), spike1Tangent)
                .build();

        Action spike1ToBasket = drive.actionBuilder(new Pose2d(spike1X, spike1Y, spike1Tangent))
                .turnTo(basket2Tangent)
                .lineToX(basket2X)
                .afterDisp(0, grabber::grab)
                .waitSeconds(0.25)
                .afterDisp(0, linearSlide::up)
                .splineTo(new Vector2d(basket2X, basket2Y), basket2Tangent)
                .turnTo(basket2Heading)
                .build();

        Action basketToPark = drive.actionBuilder(new Pose2d(basket2X, basket2Y, basket2Heading))
                .lineToX(spike2X / 2) // if grabbing the sample fails, then make sure that it doesn't get carried with the bot
                .turnTo(parkTangent)
                .splineTo(new Vector2d(spike2X, spike2Y), spike2Tangent)
                .splineTo(new Vector2d(parkX, parkY), parkTangent)
                .waitSeconds(1)
                .build();

        Action scoreSample = new InstantAction(() -> {
            grabber.down(); // for some mystical reason the grabber won't rotate forwards the first time unless this has been called first
            sleep(20);
            grabber.forward();
            sleep(1750);

            grabber.reset();
            sleep(500);

            grabber.down();
            linearSlide.down();
        });

        waitForStart();

        new Thread(() -> {
            while (!isStopRequested()) {
                linearSlide.run();
                sleep(20);
            }
        }).start();

        Actions.runBlocking(startToBasket);
        Actions.runBlocking(scoreSample);
        Actions.runBlocking(basketToSpike1);
        Actions.runBlocking(spike1ToBasket);
        Actions.runBlocking(scoreSample);
        Actions.runBlocking(basketToPark);

        while (!linearSlide.atSetPoint()) {
            sleep(20);
        }
    }
}
