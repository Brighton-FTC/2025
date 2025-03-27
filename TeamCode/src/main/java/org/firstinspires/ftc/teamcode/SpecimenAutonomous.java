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
@Autonomous
public class SpecimenAutonomous extends LinearOpMode {
    public double startX = 0;
    public double startY = 0;
    public double startHeading = Math.toRadians(0);

    public double subX = 36;
    public double subY = 0;
    public double subTangent = Math.toRadians(0);
    public double subHeading = Math.toRadians(90);

    public double spikeX = 36;
    public double spikeY = 42;
    public double spikeTangent = Math.toRadians(-90);

    public double spikeToParkHeading = Math.toRadians(180);

    public double parkX = 3;
    public double parkY = 42;
    public double parkTangent = Math.toRadians(180);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "linear_slide_motor");

        GrabberComponent grabber = new GrabberComponent(hardwareMap, "left_claw_servo", "right_claw_servo", "rotator_servo");
        grabber.grab();

        Action startToSub = drive.actionBuilder(new Pose2d(startX, startY, startHeading))
                .splineTo(new Vector2d(subX, subY), subTangent)
                .turnTo(subHeading)
                .build();

        Action subToPark = drive.actionBuilder(new Pose2d(subX, subY, subHeading))
                .afterDisp(0, linearSlide::down)
                .splineTo(new Vector2d(spikeX, spikeY), spikeTangent)
                .turnTo(spikeToParkHeading)
                .splineTo(new Vector2d(parkX, parkY), parkTangent)
                .build();

        Action scoreSpecimen = new InstantAction(() -> {
            grabber.down(); // for some mystical reason the grabber won't rotate forwards the first time unless this has been called first
            sleep(20);
            grabber.up();
            sleep(1750);

            grabber.reset();
            sleep(500);

            grabber.down();
            linearSlide.upSpecimen();
        });

        waitForStart();

        Actions.runBlocking(startToSub);
        Actions.runBlocking(scoreSpecimen);
        Actions.runBlocking(subToPark);
    }
}
