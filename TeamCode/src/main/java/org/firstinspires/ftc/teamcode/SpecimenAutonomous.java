package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "Specimen Autonomous", preselectTeleOp = "Teleop")
public class SpecimenAutonomous extends LinearOpMode {
    public static double startX = 0;
    public static double startY = 0;
    public static double startHeading = Math.toRadians(0);

    public static double subX = 28;
    public static double subY = 0;
    public static double subTangent = Math.toRadians(0);
    public static double subHeading = Math.toRadians(90);

    public static double spikeX = 42;
    public static double spikeY = -36;
    public static double spikeTangent = Math.toRadians(-90);

    public static double spikeToParkHeading = Math.toRadians(180);

    public static double parkX = 12;
    public static double parkY = -42;
    public static double parkTangent = Math.toRadians(180);
    public static double parkHeading = Math.toRadians(0);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "linear_slide_motor");

        GrabberComponent grabber = new GrabberComponent(hardwareMap, "left_claw_servo", "right_claw_servo", "rotator_servo");
        grabber.tilt();

        Action startToSub = drive.actionBuilder(new Pose2d(startX, startY, startHeading))
                .afterDisp(0, linearSlide::upSpecimen)
                .splineTo(new Vector2d(subX, subY), subTangent)
                .turnTo(subHeading)
                .build();

        Action subToPark = drive.actionBuilder(new Pose2d(subX, subY, subHeading))
                .turnTo(spikeTangent)
                .afterDisp(0, linearSlide::down)
                .splineTo(new Vector2d(spikeX, spikeY), spikeTangent)
                .turnTo(spikeToParkHeading)
                .splineTo(new Vector2d(parkX, parkY), parkTangent)
                .afterDisp(0, grabber::grab)
                .waitSeconds(0.2)
                .turnTo(parkHeading)
                .build();

        Action scoreSpecimen = new InstantAction(() -> {
            grabber.down(); // for some mystical reason the grabber won't rotate forwards the first time unless this has been called first
            sleep(20);
            grabber.forward();
            sleep(1750);
            grabber.scoreSpecimen();
            sleep(500);

            grabber.reset();
            sleep(500);

            grabber.down();
        });

        grabber.grab();

        waitForStart();

        new Thread(() -> {
            while (!isStopRequested()) {
                linearSlide.run();
                sleep(20);
            }
        }).start();

        Actions.runBlocking(startToSub);
        Actions.runBlocking(scoreSpecimen);
        Actions.runBlocking(subToPark);

        GeneralTeleop.setHeadingOffset(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
    }
}
