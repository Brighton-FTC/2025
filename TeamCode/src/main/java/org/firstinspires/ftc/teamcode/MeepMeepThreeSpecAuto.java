package org.firstinspires.ftc.teamcode;

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
@Autonomous(name = "MM 3 Specimen Autonomous")
public class MeepMeepThreeSpecAuto extends LinearOpMode {
    public static double startY = -63;
    static double UP = Math.toRadians(90);
    static double LEFT = Math.toRadians(180);

    static double DOWN = Math.toRadians(270);
    static double RIGHT = Math.toRadians(360);

    static double startX = 9;
    static double scoreX = -5;
    static double subY = -40;
    static double loop_x = 37;

    static double collectY = -67;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, UP));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startX, subY, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "vertical_slide_motor", "vertical_slide_sensor");


        GrabberComponent grabber = new GrabberComponent(hardwareMap, "claw_servo");
        grabber.grab();

        Action scoreSpec = new SequentialAction(new InstantAction(linearSlide::score), new SleepAction(0.2), new InstantAction(grabber::reset));
        Action collectSpec = new SequentialAction(new InstantAction(grabber::grab), new SleepAction(0.3), new InstantAction(linearSlide::up));
//
//        Action startToSub = drive.actionBuilder(new Pose2d(startX, startY, UP))
//                .afterDisp(0, linearSlide::up)
//                .splineToConstantHeading(new Vector2d(startX, subY), UP)
//                .build();
//
//        Vector2d collectSpec = new Vector2d(35, -50);
//
//        Action specCycleN = drive.actionBuilder(new Pose2d(startX, subY, UP))
//                .afterDisp(0, linearSlide::down)
//                .setTangent(DOWN)
//                .splineToLinearHeading(new Pose2d(35, startY-20, DOWN), DOWN)
//                .afterDisp(0, grabber::grab)
//                .afterTime(1, linearSlide::up)
//                .splineToSplineHeading(new Pose2d(startX+2, subY, UP), DOWN)
//                .build();
//
//        Action scoreToPark = drive.actionBuilder(new Pose2d(startX +2, subY, UP))
//                .afterTime(0.5, grabber::reset)
//                .splineToConstantHeading(new Vector2d(loop_x,startY+2), UP)
//                .afterTime(0, linearSlide::down)
//                .build();

        // REMEMBER: setTanget changes tanget at start, tanget parameter changes tanget at end

        double sampCollectTangent = Math.toRadians(320);

        // FIVE SPEC
        Action Spec2 = drive.actionBuilder(new Pose2d(startX, startY, UP))
                .splineToConstantHeading(new Vector2d(scoreX, subY), UP) // score
                .afterTime(0, scoreSpec)
                .waitSeconds(0.4)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(35, collectY, DOWN), DOWN)
                .afterTime(0, collectSpec)
                .waitSeconds(0.4)
                .setTangent(UP)
                .splineToSplineHeading(new Pose2d(scoreX+4, subY, UP), UP) // score
                .afterTime(0, scoreSpec)
                .waitSeconds(0.4)
                .build();

        // COLLECT THE 3 SAMPLES
        Action Sample3Collect = drive.actionBuilder(new Pose2d(startX+2, subY, UP))
                .setTangent(Math.toRadians(315))
                .splineToConstantHeading(new Vector2d((loop_x + startX + 1), (-40 + subY -1) / 2), UP)
                .splineToConstantHeading(new Vector2d(loop_x, -10), UP)
                .setTangent(sampCollectTangent)
                .splineToConstantHeading(new Vector2d(loop_x+7, -55), UP)
                .setTangent(UP)
                .splineToConstantHeading(new Vector2d(loop_x+10, -10), UP)
                .setTangent(sampCollectTangent)
                .splineToConstantHeading(new Vector2d(loop_x+17, -55), UP)
                .setTangent(UP)
                .splineToConstantHeading(new Vector2d(loop_x+27, -10), RIGHT)
                .setTangent(DOWN)
                .splineToSplineHeading(new Pose2d(loop_x+27, -55 , UP), DOWN)
                .waitSeconds(0.4)
                .build();

        // SCORE THE 3 SPEC
        Action Sample3Score = drive.actionBuilder(new Pose2d(loop_x+27, -55 , UP))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(35, collectY, DOWN), DOWN)
                .afterTime(0, collectSpec)
                .waitSeconds(0.4)
                .setTangent(UP)
                .splineToSplineHeading(new Pose2d(scoreX+8, subY, UP), UP) // score
                .afterTime(0, scoreSpec)
                .waitSeconds(0.4)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(35, collectY, DOWN), DOWN)
                .afterTime(0, collectSpec)
                .waitSeconds(0.4)
                .setTangent(UP)
                .splineToSplineHeading(new Pose2d(scoreX+12, subY, UP), UP) // score
                .afterTime(0, scoreSpec)
                .waitSeconds(0.4)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(35, collectY, DOWN), DOWN)
                .afterTime(0, collectSpec)
                .waitSeconds(0.4)
                .setTangent(UP)
                .splineToSplineHeading(new Pose2d(scoreX+16, subY, UP), UP) // score
                .afterTime(0, scoreSpec)
                .waitSeconds(0.4)
                .build();

                // PARK
        Action park = drive.actionBuilder(new Pose2d(scoreX+16, subY, UP))
                .setTangent(DOWN)
                .splineToConstantHeading(new Vector2d(loop_x,collectY), DOWN) // park
                .build();


//        grabber.toggleClaw();

        waitForStart();

        new Thread(() -> {
            while (!isStopRequested()) {
                linearSlide.run();
                sleep(20);
            }
        }).start();

        Actions.runBlocking(new SequentialAction(Spec2, Sample3Collect, Sample3Score, new InstantAction(linearSlide::down), park));

//        Actions.runBlocking(startToSub);
//        Actions.runBlocking(new SequentialAction(startToSub, scoreSpec, specCycleN, new InstantAction(linearSlide::score), scoreToPark));
//        Actions.runBlocking(specCycleN);

//        GeneralTeleop.setHeadingOffset(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
    }
}
