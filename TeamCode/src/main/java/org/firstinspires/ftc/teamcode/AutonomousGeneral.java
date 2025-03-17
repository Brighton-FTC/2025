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
@Autonomous
public class AutonomousGeneral extends LinearOpMode {
    protected MecanumDrive drive;

    public static double basket1X = 3;
    public static double basket1Y = 24;
    public static double basket1Tangent = Math.toRadians(180);
    public static double basket1Heading = Math.toRadians(-157.5);

    public static double basketToSpikeHeading = Math.toRadians(45);

    public static double spikeX = 36;
    public static double spikeY = 6;
    public static double spikeTangent = Math.toRadians(-45);

    // at the moment, error in the robot's odometry builds up over time and causes it to be a few inches off the second time it goes to the basket
    // ideally would like to fix this, but this works for now
    public static double basket2X = 3;
    public static double basket2Y = 30;
    public static double basket2Tangent = Math.toRadians(180);
    public static double basket2Heading = Math.toRadians(-157.5);

    public static double WAIT_TIME = 0.1; // for some reason the robot's actions are more consistent when you wait in between each one

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);

        drive = new MecanumDrive(hardwareMap, initialPose);

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "linear_slide_motor");

        GrabberComponent grabber = new GrabberComponent(hardwareMap, "left_claw_servo", "right_claw_servo", "rotator_servo");
        grabber.grab();

        Action startToBasket = drive.actionBuilder(initialPose)
                .afterDisp(0, linearSlide::up)
                .splineTo(new Vector2d(12, basket1Y / 2), Math.toRadians(90))
                .waitSeconds(WAIT_TIME)
                .splineTo(new Vector2d(basket1X, basket1Y), basket1Tangent)
                .waitSeconds(WAIT_TIME)
                .turnTo(basket1Heading)
                .build();

        Action basketToSpike1 = drive.actionBuilder(new Pose2d(basket1X, basket1Y, basket1Heading))
                .turnTo(Math.toRadians(basketToSpikeHeading))
                .waitSeconds(WAIT_TIME)
                .splineTo(new Vector2d(spikeX, spikeY), spikeTangent)
                .build();

        Action spike1ToBasket = drive.actionBuilder(new Pose2d(spikeX, spikeY, spikeTangent))
                .turnTo(Math.PI)
                .waitSeconds(WAIT_TIME)
                .lineToX(basket2X)
                .afterDisp(0, grabber::grab)
                .waitSeconds(0.5)
                .afterDisp(0, linearSlide::up)
                .splineTo(new Vector2d(basket2X, basket2Y), basket2Tangent)
                .turnTo(basket2Heading)
                .build();

        Action scoreSample = new InstantAction(() -> {
            grabber.down(); // for some mystical reason the grabber won't rotate forwards unless this has been called first
            sleep(20);
            grabber.forward();
            sleep(1500);

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

        while (!linearSlide.atSetPoint()) {
            sleep(20);
        }
    }
}
