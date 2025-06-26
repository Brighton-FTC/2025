package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;
@Config
@Autonomous(name = "2 Specimen Autonomous + 0HP", preselectTeleOp = "Teleop")
public class TwoSpecAuto0HP extends LinearOpMode {
    public static double startY = -72;
    double startHeading = Math.toRadians(90);

    double downTangent = Math.toRadians(270);

    double Roriginal_x = 10;
    double subY = -40;
    double loop_x = 41;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(Roriginal_x, startY, startHeading));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(Roriginal_x, subY, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "vertical_slide_motor", "vertical_slide_sensor");


        GrabberComponent grabber = new GrabberComponent(hardwareMap, "claw_servo");
        grabber.grab();

        Action cycle1 = drive.actionBuilder(new Pose2d(Roriginal_x, subY-1, startHeading))
                .setTangent(Math.toRadians(315))
                .afterDisp(0, linearSlide::down)
                .splineToSplineHeading(new Pose2d(Roriginal_x, -48, startHeading), downTangent)
                .splineToConstantHeading(new Vector2d(loop_x + 11, -19), startHeading)
                .setTangent(Math.toRadians(320))
                .splineToConstantHeading(new Vector2d(loop_x + 13, -60), startHeading)
                .splineToSplineHeading(new Pose2d(loop_x+5, startY, downTangent), startHeading)
                .build();


        Action cycle2 = drive.actionBuilder(new Pose2d(loop_x+5, startY, startHeading))
                .setTangent(Math.toRadians(315))
                .afterDisp(0, linearSlide::down)
                .splineToSplineHeading(new Pose2d(Roriginal_x, -48, startHeading), downTangent)
                .splineToConstantHeading(new Vector2d(loop_x + 13, -19), startHeading)
                .setTangent(Math.toRadians(320))
                .splineToConstantHeading(new Vector2d(loop_x + 23, -60), startHeading)
                .splineToSplineHeading(new Pose2d(loop_x+5, startY, downTangent), startHeading)
                .build();

        Action startToSub = drive.actionBuilder(new Pose2d(Roriginal_x, startY, startHeading))
                .afterDisp(0, linearSlide::up)
                .splineToConstantHeading(new Vector2d(Roriginal_x, subY-1), startHeading)
                .build();

        Action specCycle2 = drive.actionBuilder(new Pose2d(loop_x+5, startY, downTangent))
                .afterDisp(0, linearSlide::up)
                .splineToSplineHeading(new Pose2d(Roriginal_x-2, subY-3, startHeading), 270)
                .afterDisp(0, linearSlide::score)
                .build();
//        Action specCycleN = drive.actionBuilder(new Pose2d(Roriginal_x, subY-1, startHeading))
//                .afterDisp(0, linearSlide::down)
//                .setTangent(downTangent)
//                .splineToLinearHeading(new Pose2d(35, startY-20, downTangent), downTangent)
//                .afterTime(0, grabber::grab)
//                .waitSeconds(1)
//                .afterTime(0, linearSlide::up)
//                .setTangent(startHeading)
//                .splineToSplineHeading(new Pose2d(Roriginal_x-4, subY-3, startHeading), 270)
//                .build();

        Action scoreToPark = drive.actionBuilder(new Pose2d(Roriginal_x -2, subY-3, startHeading))
                .splineToConstantHeading(new Vector2d(loop_x,startY+2), startHeading)
                .build();


//        grabber.toggleClaw();

        waitForStart();

        new Thread(() -> {
            while (!isStopRequested()) {
                linearSlide.run();
                sleep(20);
            }
        }).start();

        Actions.runBlocking(startToSub);
        Actions.runBlocking(new SequentialAction(startToSub, new InstantAction(grabber::reset), cycle1, cycle2, new InstantAction(grabber::grab), specCycle2, new InstantAction(grabber::reset), new InstantAction(linearSlide::down), scoreToPark));
//        Actions.runBlocking(specCycleN);

//        GeneralTeleop.setHeadingOffset(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
    }
}
