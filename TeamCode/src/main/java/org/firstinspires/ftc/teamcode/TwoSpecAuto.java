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
@Autonomous(name = "2 Specimen Autonomous", preselectTeleOp = "Teleop")
public class TwoSpecAuto extends LinearOpMode {
    public static double startY = -72;
    double startHeading = Math.toRadians(90);

    double downTangent = Math.toRadians(270);

    double startX = 9;
    double subY = -40;
    double loop_x = 41;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startHeading));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startX, subY, startHeading));

        LinearSlideComponent linearSlide = new LinearSlideComponent(hardwareMap, "vertical_slide_motor", "vertical_slide_sensor");


        GrabberComponent grabber = new GrabberComponent(hardwareMap, "claw_servo");
        grabber.grab();

        Action scoreSpec = new SequentialAction(new InstantAction(linearSlide::score), new SleepAction(0.15), new InstantAction(grabber::reset));

        Action startToSub = drive.actionBuilder(new Pose2d(startX, startY, startHeading))
                .afterDisp(0, linearSlide::up)
                .splineToConstantHeading(new Vector2d(startX, subY-1), startHeading)
                .build();

        Vector2d collectSpec = new Vector2d(35, -50);

        Action specCycleN = drive.actionBuilder(new Pose2d(startX, subY-1, startHeading))
                .afterDisp(0, linearSlide::down)
                .setTangent(downTangent)
                .splineToLinearHeading(new Pose2d(35, startY-22, downTangent), downTangent)
                .afterTime(0, grabber::grab)
                .waitSeconds(1)
                .afterTime(0, linearSlide::up)
                .setTangent(startHeading)
                .splineToSplineHeading(new Pose2d(startX-2, subY-3, startHeading), 270)
                .build();

        Action scoreToPark = drive.actionBuilder(new Pose2d(startX -2, subY-3, startHeading))
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
        Actions.runBlocking(new SequentialAction(startToSub, scoreSpec, specCycleN, new InstantAction(linearSlide::score), new SleepAction(0.2), new InstantAction(grabber::reset), new InstantAction(linearSlide::down), scoreToPark));
//        Actions.runBlocking(specCycleN);

//        GeneralTeleop.setHeadingOffset(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
    }
}
